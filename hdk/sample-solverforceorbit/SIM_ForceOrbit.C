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

#include "SIM_ForceOrbit.h"
#include <UT/UT_DSOVersion.h>
#include <GU/GU_PrimPoly.h>
#include <PRM/PRM_Include.h>
#include <SIM/SIM_PRMShared.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_GuideShared.h>

using namespace HDK_Sample;

void
initializeSIM(void *)
{
    IMPLEMENT_DATAFACTORY(SIM_ForceOrbit);
}

SIM_ForceOrbit::SIM_ForceOrbit(const SIM_DataFactory *factory)
    : BaseClass(factory),
      SIM_OptionsUser(this)
{
}

SIM_ForceOrbit::~SIM_ForceOrbit()
{
}

static PRM_Name theGuideShowName(SIM_NAME_SHOWGUIDE, "Show Guide");
static PRM_Name theGuideSizeName(SIM_NAME_SCALE, "Scale");
static PRM_Name theGuideColorName(SIM_NAME_COLOR, "Color");

const SIM_DopDescription *
SIM_ForceOrbit::getForceOrbitDopDescription()
{
    static PRM_Name	 thePointPositionName(SIM_NAME_POINTPOSITION,
					"Point Position");
    static PRM_Name	 thePointMassName(SIM_NAME_POINTMASS,
					"Point Mass");

    // define a template for parameters to control the force's behavior
    static PRM_Template		 theTemplates[] = {
	PRM_Template(PRM_XYZ_J,		3, &thePointPositionName),
	PRM_Template(PRM_FLT_J,		1, &thePointMassName, PRMoneDefaults),
	PRM_Template()
    };

    // define a template for parameters to control the guide geometry
    static PRM_Template		 theGuideTemplates[] = {
	PRM_Template(PRM_TOGGLE,	1, &theGuideShowName, PRMoneDefaults),
	PRM_Template(PRM_FLT_J,		1, &theGuideSizeName, PRMoneDefaults),
	PRM_Template(PRM_RGB_J,		3, &theGuideColorName, PRMoneDefaults),
	PRM_Template()
    };

    static SIM_DopDescription	 theDopDescription(true,
						"hdk_orbit",
						"Orbit Force",
						SIM_FORCES_DATANAME "/Orbit",
						classname(),
						theTemplates);
    theDopDescription.setGuideTemplates(theGuideTemplates);
    theDopDescription.setDefaultUniqueDataName(1);

    return &theDopDescription;
}

SIM_Guide *
SIM_ForceOrbit::createGuideObjectSubclass() const
{   
    return new SIM_GuideShared(this, true);
}   

void
SIM_ForceOrbit::buildGuideGeometrySubclass(const SIM_RootData &root,
					   const SIM_Options &options,
					   const GU_DetailHandle &gdh,
					   UT_DMatrix4 *xform,
					   const SIM_Time &t) const
{
    // return immediately if no guide geometry should be displayed
    if( !options.hasOption(theGuideShowName.getToken()) ||
	!options.getOptionB(theGuideShowName.getToken()) )
	return;

    if( !gdh.isNull() )
    {
	GU_DetailHandleAutoWriteLock	 gdl(gdh);
	GU_Detail			*gdp = gdl.getGdp();

	// add a color attribute
#if defined(HOUDINI_11)
	GB_AttributeRef attCd = gdp->addDiffuseAttribute(GEO_PRIMITIVE_DICT);
#else
	GA_RWHandleV3 attCd(gdp->addDiffuseAttribute(GA_ATTRIB_PRIMITIVE));
#endif
	UT_Vector3 color(1, 1, 1);
	if( options.hasOption(theGuideColorName.getToken()) )
	    color = options.getOptionV3(theGuideColorName.getToken());

	// create three mutualy perpendicular lines to indicate the position
        GA_Offset ptoff = gdp->appendPointBlock(6);
	for(int axis = 0; axis < 3; ++axis)
	{
	    const GA_Offset pt0 = ptoff++;
	    const GA_Offset pt1 = ptoff++;

	    // re-position each point
	    UT_Vector3 pos(0, 0, 0);
	    pos(axis) = -0.5;
	    gdp->setPos3(pt0, pos);
	    pos(axis) = 0.5;
	    gdp->setPos3(pt1, pos);

	    // create a colored line connecting the two points
#if defined(HOUDINI_11)
	    GU_PrimPoly *poly = (GU_PrimPoly *)gdp->appendPrimitive(GEOPRIMPOLY);
#else
	    GU_PrimPoly *poly = (GU_PrimPoly *)gdp->appendPrimitive(GEO_PRIMPOLY);
#endif
	    if(attCd.isValid())
		attCd.set(poly->getMapOffset(), color);
	    poly->appendVertex(pt0);
	    poly->appendVertex(pt1);
	}
    }
    if( xform )
    {
	// scale the guide geometry
	xform->identity();
	if( options.hasOption(theGuideSizeName.getToken()) )
	{
	    fpreal scale = options.getOptionF(theGuideSizeName.getToken());
	    xform->scale(scale, scale, scale);
	}

	// position the guide geometry
	UT_Vector3 pos = getPointPosition();
	xform->translate(pos.x(), pos.y(), pos.z());
    }
}

void
SIM_ForceOrbit::getForceSubclass(const SIM_Object &,
				 const UT_Vector3 &position,
				 const UT_Vector3 &,
				 const UT_Vector3 &,
				 const fpreal mass,
				 UT_Vector3 &force,
				 UT_Vector3 &torque) const
{
    UT_Vector3	 pointposition = getPointPosition();
    fpreal	 distancesquared;

    // Note that we don't use any gravitational constant G in this
    // calculation. This is because if we used a real value of G we'd
    // have to make our point mass about 10^10 before it would exert a
    // noticeable force.
    distancesquared = distance2(position, pointposition);
    if( !SYSequalZero(distancesquared) )
    {
	force = pointposition - position;
	force *= (mass * getPointMass()) / distancesquared;
    }
    else
	force = 0.0;
    torque = 0.0;
    applyNoise(position, force);
}

void
SIM_ForceOrbit::getForceOnCircleSubclass(const SIM_Object &object,
					 const UT_Vector3 &position,
					 const UT_Vector3 &,
					 const fpreal,
					 const UT_Vector3 &velocity,
					 const UT_Vector3 &angvel,
					 const fpreal mass,
					 UT_Vector3 &force,
					 UT_Vector3 &torque) const
{
    getForceSubclass(object, position, velocity, angvel, mass, force, torque);
}

void
SIM_ForceOrbit::getForceOnSphereSubclass(const SIM_Object &object,
					 const UT_Vector3 &position,
					 const fpreal,
					 const UT_Vector3 &velocity,
					 const UT_Vector3 &angvel,
					 const fpreal mass,
					 UT_Vector3 &force,
					 UT_Vector3 &torque) const
{
    getForceSubclass(object, position, velocity, angvel, mass, force, torque);
}

