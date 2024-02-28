//
// Created by LiYifan on 2024/2/28.
//

#include "GAS_ParticleDensity.h"
#include "Particles/SIM_FluidParticle.h"
#include "kernels.h"

#include <SIM/SIM_Engine.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_GuideShared.h>
#include <SIM/SIM_ColliderLabel.h>
#include <SIM/SIM_ForceGravity.h>
#include <SIM/SIM_Time.h>
#include <SIM/SIM_Utils.h>

#include <PRM/PRM_Name.h>
#include <PRM/PRM_Template.h>
#include <PRM/PRM_Shared.h>
#include <PRM/PRM_Default.h>
#include <PRM/PRM_Utils.h>
#include <PRM/PRM_SpareData.h>

#include <UT/UT_WorkBuffer.h>
#include <UT/UT_NetMessage.h>

bool GAS_ParticleDensity::solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) {
    UT_WorkBuffer error_msg;
    if (!Solve(engine, obj, time, timestep, error_msg) || UTisstring(error_msg.buffer()))
    {
        SIM_Data::addError(obj, SIM_MESSAGE, error_msg.buffer(), UT_ERROR_ABORT);
        return false;
    }

    return true;
}

void GAS_ParticleDensity::initializeSubclass() {
    SIM_Data::initializeSubclass();
}

void GAS_ParticleDensity::makeEqualSubclass(const SIM_Data *source) {
    SIM_Data::makeEqualSubclass(source);
}

const char *GAS_ParticleDensity::DATANAME = "ParticleDensity";
const SIM_DopDescription *GAS_ParticleDensity::getDopDescription() {
    static std::array<PRM_Name, 4> Kernels = {
            PRM_Name("0", "Poly64"),
            PRM_Name("1", "Spiky"),
            PRM_Name("2", "CubicSpline"),
            PRM_Name(nullptr)
		};

    static PRM_ChoiceList CL((PRM_ChoiceListType)(PRM_CHOICELIST_SINGLE), &(Kernels[2]));
    static PRM_Name KernelName("Kernel", "Kernel");

    static std::array<PRM_Template, 2> PRMS{
            PRM_Template(PRM_INT, 1, &KernelName, 0, &CL),
            PRM_Template()
    };
    static SIM_DopDescription DESC(true,
                                   "particle_density",
                                   "Particle Density",
                                   DATANAME,
                                   classname(),
                                   PRMS.data());
    setGasDescription(DESC);
    return &DESC;
}

bool GAS_ParticleDensity::Solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep,
                                UT_WorkBuffer &error_msg) {
    if(!obj) {
        error_msg.sprintf("No object to solve");
        return false;
    }

    SIM_FluidParticle *particleData = SIM_DATA_GET(*obj, SIM_FluidParticle::DATANAME, SIM_FluidParticle);
    if(!particleData) {
        error_msg.sprintf("No fluid particle data found");
        return false;
    }
    if(!particleData->Configured)
    {
        error_msg.sprintf("Fluid particle data not configured");
        return false;
    }

    SIM_GeometryCopy *geo = SIM_DATA_GET(*obj, SIM_GEOMETRY_DATANAME, SIM_GeometryCopy);
    if (!geo)
    {
        error_msg.appendSprintf("Geometry Is Null, From %s\n", DATANAME);
        return false;
    }

    HinaPE::CubicSplineKernel<false> kernel(particleData->getTargetSpacing() * particleData->getKernelRadiusOverTargetSpacing());
    particleData->for_each_offset(
            [&](const GA_Offset &pt_off)
            {
                fpreal rho = 0.;
                {
                    // important: self is also a neighbor
                    rho += kernel.kernel(0.);
                }
                UT_Vector3 p_i = particleData->pbfParticle->positions[pt_off];
                particleData->for_each_neighbor_self(pt_off, [&](const GA_Offset &n_off, const UT_Vector3 &n_pos)
                {
                    UT_Vector3 p_j = n_pos;
                    const UT_Vector3 r = p_i - p_j;
                    rho += kernel.kernel(r.length());
                });
                particleData->pbfParticle->densities[pt_off] = rho;
            });

    particleData->commit(geo);
    return true;
}
