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

#include "SIM_RadialEmit.h"
#include <UT/UT_DSOVersion.h>
#include <UT/UT_Interrupt.h>
#include <PRM/PRM_Include.h>
#include <SIM/SIM_PRMShared.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_GeometryCopy.h>
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
    IMPLEMENT_DATAFACTORY(SIM_RadialEmit);
}

/// Standard constructor, note that BaseClass was crated by the
/// DECLARE_DATAFACTORY and provides an easy way to chain through
/// the class hierarchy.
SIM_RadialEmit::SIM_RadialEmit(const SIM_DataFactory *factory)
    : BaseClass(factory)
{
}

SIM_RadialEmit::~SIM_RadialEmit()
{
}

/// Used to automatically populate the node which will represent
/// this data type.
const SIM_DopDescription *
SIM_RadialEmit::getDopDescription()
{
    static PRM_Name theGeometryName(GAS_NAME_GEOMETRY, "Geometry");

    static PRM_Name theCenterName("center",	"Center");
    static PRM_Name theDistanceName("distance",	"Distance");
    static PRM_Name theSpeedName("speed",		"Speed");
    static PRM_Name theBirthRateName("birthrate",	"Birth Rate");
    static PRM_Name theSeedName("seed",	"Seed");

    static PRM_Default	speedDefaults[] =
    {
	PRM_Default(0),
	PRM_Default(1),
    };

    static PRM_Default	distanceDefaults[] =
    {
	PRM_Default(0),
	PRM_Default(1),
    };

    static PRM_Template		 theTemplates[] = {
	PRM_Template(PRM_STRING, 1, &theGeometryName),
	PRM_Template(PRM_XYZ_J,     3, &theCenterName),
	PRM_Template(PRM_FLT_J,     2, &theDistanceName, distanceDefaults),
	PRM_Template(PRM_FLT_J,     2, &theSpeedName, speedDefaults),
	PRM_Template(PRM_INT_J,     1, &theBirthRateName, PRMfourDefaults),
	PRM_Template(PRM_INT_J,     1, &theSeedName),
	PRM_Template()
    };

    static SIM_DopDescription	 theDopDescription(
	    true,		// Should we make a DOP?
	    "hdk_radialemit",	// Internal name of the DOP.
	    "Gas Radial Emit",		// Label of the DOP
	    "Solver",		// Default data name
	    classname(),	// The type of this DOP, usually the class.
	    theTemplates);	// Template list for generating the DOP

    // Make this a microsolver:
    setGasDescription(theDopDescription);

    return &theDopDescription;
}

bool
SIM_RadialEmit::solveGasSubclass(SIM_Engine &engine,
			SIM_Object *obj,
			SIM_Time time,
			SIM_Time timestep)
{
    SIM_GeometryCopy    *geo = 0;

    geo = getGeometryCopy(obj, GAS_NAME_GEOMETRY);
    if (!geo)
    {
	addError(obj, SIM_MISSINGDATA,
		 "Geometry", UT_ERROR_MESSAGE);
            return false;
    }

    // Assume destination is an attribute we need to modify.
    SIM_GeometryAutoWriteLock lock(geo);
    GU_Detail &gdp = lock.getGdp();
    UT_WorkBuffer  msg;
    uint			 seed = getSeed();

    // Velocity to write to.
    GA_RWHandleV3 v_h(&gdp, GA_ATTRIB_POINT, "v");
    if (v_h.isInvalid())
    {
	// While this alert can be useful, it often ends
	// up as spam...
	addError(obj, SIM_MESSAGE, "Missing attribute v", UT_ERROR_MESSAGE);
    }
    else
    {
	int		born = getBirthRate();
	UT_Vector3	center = getCenter();
	UT_Vector2	distrange = getDistance();
	UT_Vector2	speedrange = getSpeed();

	while (born --> 0)
	{
	    GA_Offset		newpt = gdp.appendPointOffset();

	    UT_Vector3		pos, vel;

	    pos.x() = SYSrandom(seed) - 0.5f;
	    pos.y() = SYSrandom(seed) - 0.5f;
	    pos.z() = SYSrandom(seed) - 0.5f;

	    // We do rejection sampling of points outside the sphere
	    // to ensure a uniform distribution!
	    if (pos.length() > 0.5)
	    {
		born++;
		continue;
	    }
	    pos.normalize();
	    vel = pos;
	    pos *= distrange.x() + SYSrandom(seed) * (distrange.y()-distrange.x());
	    pos += center;

	    gdp.setPos3(newpt, pos);

	    vel *= speedrange.x() + SYSrandom(seed) * (speedrange.y()-speedrange.x());
	    v_h.set(newpt, vel);
	}
    }

    // Successful cook
    return true;
}

