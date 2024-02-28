//
// Created by LiYifan on 2024/2/27.
//

#include "GAS_ConfigureFluidParticleData.h"
#include <Particles/SIM_FluidParticle.h>

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

bool GAS_ConfigureFluidParticleData::solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time,
                                                      SIM_Time timestep) {
    UT_WorkBuffer error_msg;
    if (!Solve(engine, obj, time, timestep, error_msg) || UTisstring(error_msg.buffer()))
    {
        SIM_Data::addError(obj, SIM_MESSAGE, error_msg.buffer(), UT_ERROR_ABORT);
        return false;
    }

    return true;
}

void GAS_ConfigureFluidParticleData::initializeSubclass() {
    SIM_Data::initializeSubclass();
}

void GAS_ConfigureFluidParticleData::makeEqualSubclass(const SIM_Data *source) {
    SIM_Data::makeEqualSubclass(source);
}

const char *GAS_ConfigureFluidParticleData::DATANAME = "Configure_Fluid_Particle_Data";
const SIM_DopDescription *GAS_ConfigureFluidParticleData::getDopDescription() {
    static std::array<PRM_Template, 1> PRMS{
            PRM_Template()
    };

    static SIM_DopDescription DESC(true,
                                   "configure_fluid_particle_data",
                                   "Configure_Fluid_Particle_Data",
                                   DATANAME,
                                   classname(),
                                   PRMS.data());
    setGasDescription(DESC);
    return &DESC;
}

bool GAS_ConfigureFluidParticleData::Solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep,
                                           UT_WorkBuffer &error_msg) {
    if (!obj)
    {
        error_msg.appendSprintf("Object Is Null, From %s\n", DATANAME);
        return false;
    }

    // std::cout << SIM_FluidParticle::DATANAME << std::endl;
    //SIM_FluidParticle *data = SIM_DATA_CAST((*obj).getNamedSubData(SIM_FluidParticle::DATANAME), SIM_FluidParticle);

    SIM_FluidParticle *data = SIM_DATA_GET(*obj, SIM_FluidParticle::DATANAME, SIM_FluidParticle);
    if (!data)
    {
        error_msg.appendSprintf("SIM_FluidParticle Is Null, From %s\n", DATANAME);
        return false;
    }

    //std::cout << data->Configured << std::endl;

    if (data->Configured)
        return true;

    data->pbfParticle = std::make_shared<PbfParticle>();

    SIM_Geometry *geo_exist = SIM_DATA_GET(*obj, SIM_GEOMETRY_DATANAME, SIM_Geometry);
    if (geo_exist)
    {
        error_msg.appendSprintf("Geometry Already Exist, From %s\n", "Configure Fluid Particle Data");
        return false;
    }

    SIM_GeometryCopy *geo = SIM_DATA_CREATE(*obj, SIM_GEOMETRY_DATANAME, SIM_GeometryCopy,
                                            SIM_DATA_RETURN_EXISTING | SIM_DATA_ADOPT_EXISTING_ON_DELETE);
    if (!geo)
    {
        error_msg.appendSprintf("Create Geometry Failed, From %s\n", "Configure Fluid Particle Data");
        return false;
    }

    {
        SIM_GeometryAutoWriteLock lock(geo);
        GU_Detail &gdp = lock.getGdp();
        SIM_FluidParticle::setup_gdp(&gdp);
    }

    data->Configured = true;

    return true;
}




