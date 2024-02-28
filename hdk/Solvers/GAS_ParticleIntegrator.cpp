//
// Created by LiYifan on 2024/2/28.
//

#include "GAS_ParticleIntegrator.h"
#include "Particles/SIM_FluidParticle.h"
#include "SIM_Hina_Generator.h"

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

bool GAS_ParticleIntegrator::solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) {
    UT_WorkBuffer error_msg;
    if (!Solve(engine, obj, time, timestep, error_msg) || UTisstring(error_msg.buffer()))
    {
        SIM_Data::addError(obj, SIM_MESSAGE, error_msg.buffer(), UT_ERROR_ABORT);
        return false;
    }

    return true;
}

void GAS_ParticleIntegrator::initializeSubclass() {
    SIM_Data::initializeSubclass();
}

void GAS_ParticleIntegrator::makeEqualSubclass(const SIM_Data *source) {
    SIM_Data::makeEqualSubclass(source);
}

const char *GAS_ParticleIntegrator::DATANAME = "ParticleIntegrator";
const SIM_DopDescription *GAS_ParticleIntegrator::getDopDescription() {
    static std::array<PRM_Template, 1> PRMS{
            PRM_Template()
    };

    static SIM_DopDescription DESC(true,
                                   "particle_integrator",
                                   "Particle Integrator",
                                   DATANAME,
                                   classname(),
                                   PRMS.data());
    setGasDescription(DESC);
    return &DESC;
}

bool GAS_ParticleIntegrator::Solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep,
                                   UT_WorkBuffer &error_msg) const {
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
    // particleData->check_data();

    {
        SIM_GeometryAutoWriteLock lock(geo);
        GU_Detail &gdp = lock.getGdp();
        GA_RWHandleV3 pos_handle = gdp.getP();
        GA_RWHandleV3 vel_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VELOCITY);
        GA_RWHandleV3 force_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_FORCE);

        GA_Size num_particles = gdp.getNumPoints();
        for (long i = 0; i < num_particles; ++i) {
            GA_Offset pt_off = gdp.appendPoint();
            UT_Vector3 pos = pos_handle.get(pt_off);
            UT_Vector3 vel = vel_handle.get(pt_off);
            pos += vel * timestep;
            pos_handle.set(pt_off, pos);
            vel += force_handle.get(pt_off) * timestep;
            vel_handle.set(pt_off, vel);
        }
    }

    particleData->load(geo);

    return true;
}
