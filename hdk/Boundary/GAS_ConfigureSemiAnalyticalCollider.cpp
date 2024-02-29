//
// Created by LiYifan on 2024/2/29.
//

#include "GAS_ConfigureSemiAnalyticalCollider.h"
#include "SIM_SemiAnalyticalCollider.h"

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

bool GAS_ConfigureSemiAnalyticalCollider::solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time,
                                                           SIM_Time timestep) {
    UT_WorkBuffer error_msg;
    if (!Solve(engine, obj, time, timestep, error_msg) || UTisstring(error_msg.buffer()))
    {
        SIM_Data::addError(obj, SIM_MESSAGE, error_msg.buffer(), UT_ERROR_ABORT);
        return false;
    }

    return true;
}

void GAS_ConfigureSemiAnalyticalCollider::initializeSubclass() {
    SIM_Data::initializeSubclass();
}

void GAS_ConfigureSemiAnalyticalCollider::makeEqualSubclass(const SIM_Data *source) {
    SIM_Data::makeEqualSubclass(source);
}

const char *GAS_ConfigureSemiAnalyticalCollider::DATANAME = "ConfigureSemiAnalyticalCollider";
const SIM_DopDescription *GAS_ConfigureSemiAnalyticalCollider::getDopDescription() {
    static std::array<PRM_Template, 1> PRMS{
            PRM_Template()
    };

    static SIM_DopDescription DESC(true,
                                   "configure_semi_analytical_collider",
                                   "Configure Semi Analytical Collider",
                                   DATANAME,
                                   classname(),
                                   PRMS.data());
    setGasDescription(DESC);
    return &DESC;
}

bool GAS_ConfigureSemiAnalyticalCollider::Solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep,
                                                UT_WorkBuffer &error_msg) {
    if (!obj) {
        error_msg.appendSprintf("Object Is Null, From %s\n", DATANAME);
        return false;
    }

    SIM_SemiAnalyticalCollider *collider = SIM_DATA_CREATE(*obj, SIM_SemiAnalyticalCollider::DATANAME,
                                                           SIM_SemiAnalyticalCollider,
                                                           SIM_DATA_RETURN_EXISTING |
                                                           SIM_DATA_ADOPT_EXISTING_ON_DELETE);
    if(!collider) {
        error_msg.appendSprintf("Failed to create SemiAnalyticalCollider, From %s\n", DATANAME);
        return false;
    }
    if(collider->Configured)
        return true;

    collider->boundary = std::make_shared<SemiAnalyticalBoundary>();
    collider->Configured = true;
    return true;
}
