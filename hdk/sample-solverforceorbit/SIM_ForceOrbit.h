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

/// @file SIM_ForceOrbit.h
/// This example creates a custom force DOP.
///
/// This class will add a new DOP node called Orbit Force.  This node will
/// attach a SIM_ForceOrbit instance on DOP objects.  Solvers will discover
/// this instance when iterating over the applied forces and can call one of
/// getForce*() methods to find out the applied force and torque values.
/// Houdini will also use the createGuideObject() method to display guide
/// geometry.

#ifndef __SIM_ForceOrbit_h__
#define __SIM_ForceOrbit_h__

#include <SIM/SIM_Force.h>
#include <SIM/SIM_OptionsUser.h>

#define SIM_NAME_POINTMASS	"pointmass"
#define SIM_NAME_POINTPOSITION	"pointposition"

namespace HDK_Sample {

/// This is an implementation of the SIM_Force interface. This
/// implementation returns a force value that is proportional to the
/// square of the distance from a point in space (as if there were an
/// infinitely small massive body at that location).
class SIM_ForceOrbit : public SIM_Force,
		       public SIM_OptionsUser
{
public:
    /// Controls the position of the point we are orbiting.
    GETSET_DATA_FUNCS_V3(SIM_NAME_POINTPOSITION, PointPosition);
    /// Controls the mass of the point we are orbiting.
    GETSET_DATA_FUNCS_F(SIM_NAME_POINTMASS, PointMass);

protected:
    /// Declare a protected constructor to ensure the data factory is used to
    /// create new instances of SIM_ForceOrbit.
    explicit		 SIM_ForceOrbit(const SIM_DataFactory *factory);
                        ~SIM_ForceOrbit() override;

    /// Override this to return a SIM_GuideShared as the guide geometry looks
    /// the same for all objects.
    SIM_Guide           *createGuideObjectSubclass() const override;

    /// We override this method to create the attractor geometry.
    void                 buildGuideGeometrySubclass(const SIM_RootData &root,
						    const SIM_Options &options,
						    const GU_DetailHandle &gdh,
						    UT_DMatrix4 *xform,
						    const SIM_Time &t
                                                    ) const override;

    /// Calculates the force on a point. The force depends only on the
    /// mass of the point.
    void                 getForceSubclass(const SIM_Object &object,
					const UT_Vector3 &position,
					const UT_Vector3 &velocity,
					const UT_Vector3 &angvel,
					const fpreal mass,
					UT_Vector3 &force,
					UT_Vector3 &torque) const override;
    /// Calculates the force on a surface. The force depends only
    /// on the mass, so this function simply calls getForceSubclass(),
    /// ignoring the circle parameters.
    void                 getForceOnCircleSubclass(const SIM_Object &object,
					const UT_Vector3 &position,
					const UT_Vector3 &normal,
					const fpreal radius,
					const UT_Vector3 &velocity,
					const UT_Vector3 &angvel,
					const fpreal mass,
					UT_Vector3 &force,
					UT_Vector3 &torque) const override;
    /// Calculates the force on a volume. The force depends only
    /// on the mass, so this function simply calls getForceSubclass(),
    /// ignoring the sphere parameters.
    void                 getForceOnSphereSubclass(const SIM_Object &object,
					const UT_Vector3 &position,
					const fpreal radius,
					const UT_Vector3 &velocity,
					const UT_Vector3 &angvel,
					const fpreal mass,
					UT_Vector3 &force,
					UT_Vector3 &torque) const override;

private:
    /// Get the SIM_DopDescription for creating an orbit force.
    static const SIM_DopDescription	*getForceOrbitDopDescription();

    /// This macro adds the standard getCastToType() method needed by
    /// DECLARE_DATAFACTORY()
    DECLARE_STANDARD_GETCASTTOTYPE();
    /// This macro defines all the static functions needed to declare a
    /// SIM_DataFactory for SIM_ForceOrbit.
    DECLARE_DATAFACTORY(SIM_ForceOrbit,
			SIM_Force,
			"Orbit Force",
			getForceOrbitDopDescription());
};

} // End HDK_Sample namespace

#endif

