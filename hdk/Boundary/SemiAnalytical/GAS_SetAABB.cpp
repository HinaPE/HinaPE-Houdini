//
// Created by LiYifan on 2024/2/29.
//

#include "GAS_SetAABB.h"
#include "Particles/SIM_FluidParticle.h"
#include "Boundary/SIM_SemiAnalyticalCollider.h"

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

bool GAS_SetAABB::solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) {
    UT_WorkBuffer error_msg;
    if (!Solve(engine, obj, time, timestep, error_msg) || UTisstring(error_msg.buffer()))
    {
        SIM_Data::addError(obj, SIM_MESSAGE, error_msg.buffer(), UT_ERROR_ABORT);
        return false;
    }

    return true;
}

void GAS_SetAABB::initializeSubclass() {
    SIM_Data::initializeSubclass();
}

void GAS_SetAABB::makeEqualSubclass(const SIM_Data *source) {
    SIM_Data::makeEqualSubclass(source);
}

const char *GAS_SetAABB::DATANAME = "SetAABB";
const SIM_DopDescription *GAS_SetAABB::getDopDescription() {
    static std::array<PRM_Template, 1> PRMS{
            PRM_Template()
    };

    static SIM_DopDescription DESC(true,
                                   "set_aabb",
                                   "Set AABB",
                                   DATANAME,
                                   classname(),
                                   PRMS.data());

    return &DESC;
}

bool GAS_SetAABB::Solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep,
                        UT_WorkBuffer &error_msg) const {
    if (!obj)
    {
        error_msg.appendSprintf("Object Is Null, From %s\n", DATANAME);
        return false;
    }

    SIM_FluidParticle *particle = SIM_DATA_GET(*obj, SIM_FluidParticle::DATANAME, SIM_FluidParticle);
    if (!particle)
    {
        error_msg.appendSprintf("Object Is Not A Fluid Particle, From %s\n", DATANAME);
        return false;
    }
    if(!particle->Configured)
    {
        error_msg.appendSprintf("Fluid Particle Is Not Configured, From %s\n", DATANAME);
        return false;
    }

    SIM_SemiAnalyticalCollider *collider = SIM_DATA_GET(*obj, SIM_SemiAnalyticalCollider::DATANAME, SIM_SemiAnalyticalCollider);
    if(!collider)
    {
        error_msg.appendSprintf("SemiAnalyticalCollider Is Null, From %s\n", DATANAME);
        return false;
    }
    if(!collider->Configured)
    {
        error_msg.appendSprintf("SemiAnalyticalCollider Not Configured Yet, From %s\n", DATANAME);
        return false;
    }

/*    particle->pbfParticle->mQueryAABB.clear();
    collider->boundary->mQueriedAABB.clear();*/

    fpreal kernel_radius = particle->getTargetSpacing() * particle->getKernelRadiusOverTargetSpacing() * 0.9;

    // set up point aabb
    particle->for_each_offset(
            [&](const GA_Offset &pt_off)
            {
                UT_Vector3 pos = particle->pbfParticle->positions[pt_off];
                UT_Vector3 aabb_min = pos - kernel_radius;
                UT_Vector3 aabb_max = pos + kernel_radius;
                AlignedBox point_aabb(aabb_min, aabb_max);
                particle->pbfParticle->mQueryAABB[pt_off] = point_aabb;
            });

    // set up triangle aabb
    collider->for_each_triangle([&](const GA_Offset &prim_off,const std::vector<size_t> &triangle){

        if(triangle.empty()) {
            std::cout << "Warning: triangle is empty" << std::endl;
            return; // Skip computation for this triangle
        }

        size_t v0 = triangle[0];
        size_t v1 = triangle[1];
        size_t v2 = triangle[2];

        if (v0 >= collider->boundary->vertices.size() ||
            v1 >= collider->boundary->vertices.size() ||
            v2 >= collider->boundary->vertices.size()) {
            std::cout << "Warning: index out of bounds" << std::endl;
            return; // Skip computation for this triangle
        }

        UT_Vector3 V0 = collider->boundary->vertices[v0];
        UT_Vector3 V1 = collider->boundary->vertices[v1];
        UT_Vector3 V2 = collider->boundary->vertices[v2];

        UT_Vector3 aabb_min = UT_Vector3(
                std::min(std::min(V0.x(), V1.x()), V2.x()),
                std::min(std::min(V0.y(), V1.y()), V2.y()),
                std::min(std::min(V0.z(), V1.z()), V2.z())
        );

        UT_Vector3 aabb_max = UT_Vector3(
                std::max(std::max(V0.x(), V1.x()), V2.x()),
                std::max(std::max(V0.y(), V1.y()), V2.y()),
                std::max(std::max(V0.z(), V1.z()), V2.z())
        );

        AlignedBox triangle_aabb(aabb_min, aabb_max);
        collider->boundary->mQueriedAABB[prim_off] = triangle_aabb;
    });

    //particle->check_AABB();
    //collider->check_AABB();
    return true;
}

