//
// Created by LiYifan on 2024/2/28.
//

#include "GAS_ParticleViscosity.h"
#include "Particles/SIM_FluidParticle.h"

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

bool GAS_ParticleViscosity::solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) {
    UT_WorkBuffer error_msg;
    if (!Solve(engine, obj, time, timestep, error_msg) || UTisstring(error_msg.buffer()))
    {
        SIM_Data::addError(obj, SIM_MESSAGE, error_msg.buffer(), UT_ERROR_ABORT);
        return false;
    }

    return true;
}

void GAS_ParticleViscosity::initializeSubclass() {
    SIM_Data::initializeSubclass();
}

void GAS_ParticleViscosity::makeEqualSubclass(const SIM_Data *source) {
    SIM_Data::makeEqualSubclass(source);
}

const char *GAS_ParticleViscosity::DATANAME = "ParticleViscosity";
const SIM_DopDescription *GAS_ParticleViscosity::getDopDescription() {
    static PRM_Name Viscosity("Viscosity", "Viscosity");
    static PRM_Default ViscosityDefault(0.05);

    static PRM_Name Iteration("Iteration", "Iteration");
    static PRM_Default IterationDefault(3);

    static std::array<PRM_Template, 3> PRMS{
            PRM_Template(PRM_FLT, 1, &Viscosity, &ViscosityDefault),
            PRM_Template(PRM_INT, 1, &Iteration, &IterationDefault),
            PRM_Template()
    };

    static SIM_DopDescription DESC(true,
                                   "particle_viscosity",
                                   "Particle Viscosity",
                                   DATANAME,
                                   classname(),
                                   PRMS.data());
    setGasDescription(DESC);
    return &DESC;
}

bool GAS_ParticleViscosity::Solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep,
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

    fpreal h = particleData->getTargetSpacing();
    fpreal viscosity = getViscosity();
    int iterNum = getIteration();

    std::map<GA_Offset,UT_Vector3> mVelOld;
    std::map<GA_Offset,UT_Vector3> mVelBuf;

    mVelOld = particleData->pbfParticle->velocities;
    for (int t = 0; t < iterNum; t++)
    {
        mVelBuf = particleData->pbfParticle->velocities;

        particleData->for_each_offset([&](const GA_Offset &pt_off){
            UT_Vector3 dv_i = UT_Vector3(0, 0, 0);
            UT_Vector3 p_i = particleData->pbfParticle->positions[pt_off];
            UT_Vector3 vel = mVelBuf[pt_off];
            fpreal totalWeight = 0.0f;

            particleData->for_each_neighbor_self(pt_off, [&](const GA_Offset &n_off, const UT_Vector3 &n_pos)
            {
                UT_Vector3 p_j = n_pos;
                const UT_Vector3 r = p_i - p_j;
                fpreal rLen = r.length();

                if(rLen > std::numeric_limits<fpreal>::epsilon())
                {
                    fpreal weight = IV_Weight(rLen, h);
                    totalWeight += weight;
                    dv_i += weight * mVelBuf[n_off];
                }
            });

            fpreal b = timestep * viscosity / h;
            b = totalWeight < std::numeric_limits<fpreal>::epsilon() ? 0.0f : b;

            dv_i /= totalWeight;

            particleData->pbfParticle->velocities[pt_off] = mVelOld[pt_off] / (1.0f + b) + dv_i * b / (1.0f + b);
        });
    }

    particleData->commit(geo);

    return true;
}

fpreal GAS_ParticleViscosity::IV_Weight(const fpreal r, const fpreal h) {
    fpreal q = r / h;
    if (q > 1.0f) return 0.0;
    else {
        const fpreal d = 1.0f - q;
        const fpreal hh = h*h;
        return 45.0f / (13.0f * (fpreal)M_PI * hh *h) *d;
    }
}
