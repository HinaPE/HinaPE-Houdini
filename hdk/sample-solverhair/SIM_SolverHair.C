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

#include "SIM_SolverHair.h"
#include <UT/UT_DSOVersion.h>
#include <GU/GU_PrimPoly.h>
#include <PRM/PRM_Include.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_DataFilter.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_Engine.h>
#include <SIM/SIM_Force.h>

using namespace HDK_Sample;

void
initializeSIM(void *)
{
    IMPLEMENT_DATAFACTORY(SIM_SolverHair);
}

namespace HDK_Sample {

class SIM_HairForceCallback : public SIM_PointForceCallback,
			      public SIM_EachDataCallback
{
public:
#if defined(HOUDINI_11)
			 SIM_HairForceCallback(GU_Detail &gdp,
					const SIM_Object &object,
					const UT_DMatrix4 &xform,
					fpreal timestep,
					const GB_AttributeRef &massoffset,
					const GB_AttributeRef &veloffset);
#else
			 SIM_HairForceCallback(GU_Detail &gdp,
					const SIM_Object &object,
					const UT_DMatrix4 &xform,
					fpreal timestep,
					const GA_RWHandleF &masshandle,
					const GA_RWHandleV3 &velhandle);
#endif
                        ~SIM_HairForceCallback() override;

    void                 callbackConst(const SIM_Data *data,
				       const char *name) override;
    void                 forceCallbackOffset(GA_Offset ptoff,
				       const UT_Vector3 &force,
				       const UT_Vector3 &torque) override;

private:
    GU_Detail		&myGdp;
    const SIM_Object	&myObject;
    const UT_DMatrix4	 myTransform;
    fpreal		 myTimestep;
    GA_RWHandleF	 myMassHandle;
    GA_RWHandleV3	 myVelHandle;
};

} // namespace HDK_Sample

SIM_HairForceCallback::SIM_HairForceCallback(GU_Detail &gdp,
		const SIM_Object &object,
		const UT_DMatrix4 &xform,
		fpreal timestep,
		const GA_RWHandleF &masshandle,
		const GA_RWHandleV3 &velhandle)
    : myGdp(gdp)
    , myObject(object)
    , myTransform(xform)
    , myTimestep(timestep)
    , myMassHandle(masshandle)
    , myVelHandle(velhandle)
{
}

SIM_HairForceCallback::~SIM_HairForceCallback()
{
}

void
SIM_HairForceCallback::callbackConst(const SIM_Data *data, const char *)
{
    const SIM_Force	*force = SIM_DATA_CASTCONST(data, SIM_Force);
    GU_DetailHandle	 gdh;

    gdh.allocateAndSet(&myGdp, false);
    force->getPointForces(*this, myObject, gdh, myTransform, 0, 0, false);
}

void
SIM_HairForceCallback::forceCallbackOffset(GA_Offset ptoff,
				     const UT_Vector3 &force,
				     const UT_Vector3 &)
{
    float mass = myMassHandle.get(ptoff);
    myVelHandle.set(ptoff, myVelHandle.get(ptoff) + (force/mass)*myTimestep);
}

SIM_SolverHair::SIM_SolverHair(const SIM_DataFactory *factory)
    : BaseClass(factory),
      SIM_OptionsUser(this)
{
}

SIM_SolverHair::~SIM_SolverHair()
{
}

const SIM_DopDescription *
SIM_SolverHair::getSolverHairDopDescription()
{
    static PRM_Template		 theTemplates[] = {
	PRM_Template()
    };

    static SIM_DopDescription	 theDopDescription(true,
						   "hdk_hairsolver",
						   "Hair Solver",
						   SIM_SOLVER_DATANAME,
						   classname(),
						   theTemplates);

    return &theDopDescription;
}

void
SIM_SolverHair::solveHair(SIM_GeometryCopy &hairgeo,
			  const SIM_ObjectArray &srcobjs,
			  const SIM_Object &object,
			  const SIM_Time &timestep) const
{
    SIM_GeometryAutoWriteLock lock(&hairgeo);
    GU_Detail &gdp = lock.getGdp();

    UT_DMatrix4 xform;
    hairgeo.getTransform(xform);
    for (exint geonum = 0; geonum < srcobjs.entries(); geonum++)
    {
	const SIM_Object *sourceobj = srcobjs(geonum);
	const SIM_Geometry *sourcegeo = sourceobj->getGeometry();
	if (!sourcegeo || sourcegeo->getGeometry().isNull())
	    continue;

	GU_DetailHandleAutoReadLock sourcegdl(sourcegeo->getGeometry());
	const GU_Detail *sourcegdp = sourcegdl.getGdp();
	UT_DMatrix4 sourcexform;
	
	SIMgetGeometryTransform(sourcexform, *sourceobj);
#if defined(HOUDINI_11)
	const UT_Vector3	 zero(0.0, 0.0, 0.0);
	const float		 one = 1.0;
	massoffset = gdp.addPointAttrib(
		    gdp.getStdAttributeName(GEO_ATTRIBUTE_MASS),
		    sizeof(float), GB_ATTRIB_FLOAT, &one);
	veloffset = gdp.addPointAttrib(
		    gdp.getStdAttributeName(GEO_ATTRIBUTE_VELOCITY),
		    sizeof(UT_Vector3), GB_ATTRIB_VECTOR, &zero);
#else
	GA_RWHandleF masshandle(gdp.addFloatTuple(GA_ATTRIB_POINT,
		    gdp.getStdAttributeName(GEO_ATTRIBUTE_MASS),
		    1, GA_Defaults(1.0)));
	GA_RWAttributeRef velref = gdp.addFloatTuple(GA_ATTRIB_POINT,
		    gdp.getStdAttributeName(GEO_ATTRIBUTE_VELOCITY),
		    3, GA_Defaults(0.0));
	velref.setTypeInfo(GA_TYPE_VECTOR);	// Transform as vector
        GA_RWHandleV3 velhandle(velref);
#endif

	// Apply all forces and move the hairs according to those forces.
	SIM_HairForceCallback callback(gdp, object, xform, timestep,
				       masshandle, velhandle);
	object.forEachConstSubData(callback,
				   SIM_DataFilterByType("SIM_Force"),
				   SIM_FORCES_DATANAME,
				   SIM_DataFilterNone());

	// Integrate our velocities.
        {
            GA_Offset hairptoff;
	    GA_FOR_ALL_PTOFF((&gdp), hairptoff)
	    {
	        UT_Vector3 dp = velhandle.get(hairptoff);
	        dp *= timestep;

	        gdp.setPos3(hairptoff, gdp.getPos3(hairptoff) + dp);
	    }
        }
	
	// Now pull all the points along to make sure the hair length
	// remains constant.
        GA_Iterator primit(gdp.getPrimitiveRange());
        GA_Offset sourceptoff;
	GA_FOR_ALL_PTOFF(sourcegdp, sourceptoff)
	{
	    if (primit.atEnd())
		break;

	    GA_Primitive *hairpoly = gdp.getPrimitiveList().get(*primit);
	    if (hairpoly->getTypeId() == GA_PRIMPOLY &&
		hairpoly->getVertexCount() == 10)
	    {
	        UT_Vector3 startpos = sourcegdp->getPos3(sourceptoff);
	        startpos *= sourcexform;

		GA_Offset hairpt0off = hairpoly->getPointOffset(0);
		GA_Offset hairpt1off = hairpoly->getPointOffset(1);
		// Calculate the change in position we need to enforce
		// our length constraints. Then calculate the change in
		// velocity inherent in this change in position, using
		// dp = vt, we get dv = dp/t.
		UT_Vector3 dp = startpos - gdp.getPos3(hairpt0off);
		UT_Vector3 dv = dp / timestep;
		UT_Vector3 vel = velhandle.get(hairpt0off);
		vel += dv;
		// Apply drag. Just scale down the velocity.
		vel *= 0.8;
		velhandle.set(hairpt0off, vel);
		gdp.setPos3(hairpt0off, gdp.getPos3(hairpt0off) + dp);
                GA_Offset hairpt2off;
		for (int ptnum = 1; ptnum < 10; ptnum++)
		{
		    if (ptnum < 9)
			hairpt2off = hairpoly->getPointOffset(ptnum + 1);
		    UT_Vector3 midline = gdp.getPos3(hairpt2off) - gdp.getPos3(hairpt0off);
		    // Calculate the change in position we need to enforce
		    // our length constraints. Then calculate the change in
		    // velocity inherent in this change in position, using
		    // dp = vt, we get dv = dp/t.
		    dp = midline * (0.1 / midline.length()) +
			 gdp.getPos3(hairpt0off) - gdp.getPos3(hairpt1off);
		    dv = dp / timestep;
		    vel = velhandle.get(hairpt1off);
		    vel += dv;
		    // Apply drag. Just scale down the velocity.
		    vel *= 0.8;
		    velhandle.set(hairpt1off, vel);
		    gdp.setPos3(hairpt1off, gdp.getPos3(hairpt1off) + dp);
		    hairpt0off = hairpt1off;
		    hairpt1off = hairpt2off;
		}
	    }
	    ++primit;
	}
    }
}

void
SIM_SolverHair::createHairFromSource(SIM_GeometryCopy &hairgeo,
				     const SIM_ObjectArray &srcobjs) const
{
    SIM_GeometryAutoWriteLock lock(&hairgeo);
    GU_Detail &gdp = lock.getGdp();

    for (exint geonum = 0; geonum < srcobjs.entries(); geonum++)
    {
	const SIM_Object *sourceobj = srcobjs(geonum);
	const SIM_Geometry *sourcegeo = sourceobj->getGeometry();
	if(!sourcegeo || sourcegeo->getGeometry().isNull())
	    continue;

	GU_DetailHandleAutoReadLock sourcegdl(sourcegeo->getGeometry());
	const GU_Detail *sourcegdp = sourcegdl.getGdp();
	UT_DMatrix4 sourcexform;

	SIMgetGeometryTransform(sourcexform, *sourceobj);
        GA_Offset sourceptoff;
	GA_FOR_ALL_PTOFF(sourcegdp, sourceptoff)
	{
	    UT_Vector4 startpos = sourcegdp->getPos4(sourceptoff);
	    startpos *= sourcexform;
	    GU_PrimPoly *newpoly = GU_PrimPoly::build(&gdp, 0, GU_POLY_OPEN, 0);
	    for (int i = 0; i < 10; i++)
	    {
		GA_Offset newptoff = gdp.appendPointOffset();
		gdp.setPos3(newptoff, startpos.x(),
			    startpos.y() + (fpreal)i * 0.1,
			    startpos.z());
		newpoly->appendVertex(newptoff);
	    }
	}
    }
}

SIM_Solver::SIM_Result
SIM_SolverHair::solveSingleObjectSubclass(SIM_Engine & /*engine*/,
					  SIM_Object &object,
					  SIM_ObjectArray &,
					  const SIM_Time &timestep,
					  bool newobject)
{
    SIM_GeometryCopy			*hairgeo = 0;
    SIM_ObjectArray			 sourceobjects;

    if( newobject )
    {
	if( !SIM_DATA_GET(object, SIM_GEOMETRY_DATANAME, SIM_GeometryCopy) )
	{
	    hairgeo = SIM_DATA_CREATE(object, SIM_GEOMETRY_DATANAME,
				      SIM_GeometryCopy, 0);
	    object.getAffectors(sourceobjects, "SIM_RelationshipSource");
	    if( hairgeo && sourceobjects.entries() > 0 )
		createHairFromSource(*hairgeo, sourceobjects);
	}
    }
    else
    {
	hairgeo = SIM_DATA_GET(object, SIM_GEOMETRY_DATANAME, SIM_GeometryCopy);
	if( hairgeo )
	{
	    object.getAffectors(sourceobjects, "SIM_RelationshipSource");
	    if( hairgeo && sourceobjects.entries() > 0 )
		solveHair(*hairgeo, sourceobjects, object, timestep);
	}
    }

    return hairgeo ? SIM_SOLVER_SUCCESS : SIM_SOLVER_FAIL;
}

