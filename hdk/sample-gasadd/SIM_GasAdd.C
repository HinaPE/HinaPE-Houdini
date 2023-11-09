/*
 * Copyright (c) 2023
 *	Side Effects Software Inc.  All rights reserved.
 *
 * Redistribution and use of Houdini Development Kit samples in source and
 * binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. The name of Side Effects Software may not be used to endorse or
 *    promote products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY SIDE EFFECTS SOFTWARE `AS IS' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO EVENT SHALL SIDE EFFECTS SOFTWARE BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *----------------------------------------------------------------------------
 */

#include "SIM_GasAdd.h"
#include <UT/UT_DSOVersion.h>
#include <UT/UT_Interrupt.h>
#include <PRM/PRM_Include.h>
#include <SIM/SIM_PRMShared.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_FieldSampler.h>
#include <SIM/SIM_ScalarField.h>
#include <SIM/SIM_VectorField.h>
#include <SIM/SIM_MatrixField.h>
#include <SIM/SIM_Object.h>
#include <GAS/GAS_SubSolver.h>

using namespace HDK_Sample;

///
/// This is the hook that Houdini grabs from the dll to link in
/// this.  As such, it merely has to implement the data factory
/// for this node.
///
void
initializeSIM(void *)
{
    IMPLEMENT_DATAFACTORY(SIM_GasAdd);
}

/// Standard constructor, note that BaseClass was crated by the
/// DECLARE_DATAFACTORY and provides an easy way to chain through
/// the class hierarchy.
SIM_GasAdd::SIM_GasAdd(const SIM_DataFactory *factory)
    : BaseClass(factory)
{
}

SIM_GasAdd::~SIM_GasAdd()
{
}

/// Used to automatically populate the node which will represent
/// this data type.
const SIM_DopDescription *
SIM_GasAdd::getDopDescription()
{
    static PRM_Name	theDstFieldName(GAS_NAME_FIELDDEST, "Dest Field");
    static PRM_Name	theSrcFieldName(GAS_NAME_FIELDSOURCE, "Source Field");

    static PRM_Template		 theTemplates[] = {
	PRM_Template(PRM_STRING, 1, &theDstFieldName),
	PRM_Template(PRM_STRING, 1, &theSrcFieldName),
	PRM_Template()
    };

    static SIM_DopDescription	 theDopDescription(
	    true,		// Should we make a DOP?
	    "hdk_gasadd",	// Internal name of the DOP.
	    "Gas Add",		// Label of the DOP
	    "Solver",		// Default data name
	    classname(),	// The type of this DOP, usually the class.
	    theTemplates);	// Template list for generating the DOP

    return &theDopDescription;
}

bool
SIM_GasAdd::solveGasSubclass(SIM_Engine &engine,
			SIM_Object *obj,
			SIM_Time time,
			SIM_Time timestep)
{
    SIM_ScalarField	*srcscalar, *dstscalar;
    SIM_VectorField	*srcvector, *dstvector;
    SIM_MatrixField	*srcmatrix, *dstmatrix;

    SIM_DataArray	 src, dst;
    int			 i, j, k;

    getMatchingData(src, obj, GAS_NAME_FIELDSOURCE);
    getMatchingData(dst, obj, GAS_NAME_FIELDDEST);

    // Now for each pair of source and dst fields, we want to add
    // src to dst.  We want to support scalar, vector, and matrix fields,
    // but only compatible operations.  We can determine what type we
    // have via casting.
    for (i = 0; i < dst.entries(); i++)
    {
	// Check to see if we exceeded our src list.
	if (i >= src.entries())
	{
	    addError(obj, SIM_MESSAGE, "Fewer source fields than destination fields.", UT_ERROR_WARNING);
	    break;
	}

	// Try each casting option.
	dstscalar = SIM_DATA_CAST(dst(i), SIM_ScalarField);
	srcscalar = SIM_DATA_CAST(src(i), SIM_ScalarField);

	dstvector = SIM_DATA_CAST(dst(i), SIM_VectorField);
	srcvector = SIM_DATA_CAST(src(i), SIM_VectorField);

	dstmatrix = SIM_DATA_CAST(dst(i), SIM_MatrixField);
	srcmatrix = SIM_DATA_CAST(src(i), SIM_MatrixField);

	if (dstscalar && srcscalar)
	{
	    addFields(dstscalar->getField(), srcscalar->getField());
	}

	if (dstvector && srcvector)
	{
	    for (j = 0; j < 3; j++)
		addFields(dstvector->getField(j), srcvector->getField(j));
	}

	if (dstmatrix && srcmatrix)
	{
	    for (j = 0; j < 3; j++)
		for (k = 0; k < 3; k++)
		    addFields(dstmatrix->getField(j, k), srcmatrix->getField(j, k));
	}

	// Make sure we are flagged as dirty
	if (dstscalar)
	    dstscalar->pubHandleModification();
	if (dstvector)
	    dstvector->pubHandleModification();
	if (dstmatrix)
	    dstmatrix->pubHandleModification();
    }

    // Successful cook
    return true;
}

void
SIM_GasAdd::addFieldsPartial(SIM_RawField *dst, const SIM_RawField *src, const UT_JobInfo &info)
{
    UT_VoxelArrayIteratorF	vit;
    UT_Interrupt		*boss = UTgetInterrupt();

    // Initialize our iterator to run over our destination field.
    vit.setArray(dst->fieldNC());

    // When we complete each tile the tile is tested to see if it can be
    // compressed, ie, is now constant.  If so, it is compressed.
    vit.setCompressOnExit(true);

    // Restrict our iterator only over part of the range.  Using the
    // info parameters means each thread gets its own subregion.
    vit.setPartialRange(info.job(), info.numJobs());

    // Create a sampler for the source field.
    SIM_ScalarFieldSampler srcsampler(dst, src);
    float srcval;

    // Visit every voxel of the destination array.
    for (vit.rewind(); !vit.atEnd(); vit.advance())
    {
        if (vit.isStartOfTile())
        {
            if (boss->opInterrupt())
                break;

            // Check if both source and destination tiles are constant.
            if (vit.isTileConstant() &&
                srcsampler.isTileConstant(vit, srcval))
            {
                // If both are constant, we can process the whole tile at
                // once. We call skipToEndOfTile() here so that the loop's
                // call to advance() will move us to the next tile.
                vit.getTile()->makeConstant( vit.getValue() + srcval );
                vit.skipToEndOfTile();
                continue;
            }
        }

        // Write out the sum of the two fields. Instead of using the
        // iterator, we could also have built a UT_VoxelRWProbeF.
        float srcval = srcsampler.getValue(vit);
        vit.setValue( vit.getValue() + srcval );
    }
}
