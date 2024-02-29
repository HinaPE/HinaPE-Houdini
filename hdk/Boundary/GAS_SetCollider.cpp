//
// Created by LiYifan on 2024/2/29.
//

#include "GAS_SetCollider.h"
#include "SIM_SemiAnalyticalCollider.h"
#include "Geometry/TriangleMesh3.hpp"

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

#include <GEO/GEO_PrimPoly.h>

bool GAS_SetCollider::solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) {
    UT_WorkBuffer error_msg;
    if (!Solve(engine, obj, time, timestep, error_msg) || UTisstring(error_msg.buffer()))
    {
        SIM_Data::addError(obj, SIM_MESSAGE, error_msg.buffer(), UT_ERROR_ABORT);
        return false;
    }

    return true;
}

void GAS_SetCollider::initializeSubclass() {
    SIM_Data::initializeSubclass();
}

void GAS_SetCollider::makeEqualSubclass(const SIM_Data *source) {
    SIM_Data::makeEqualSubclass(source);
}

const char *GAS_SetCollider::DATANAME = "SetCollider";
const SIM_DopDescription *GAS_SetCollider::getDopDescription() {
    static std::array<PRM_Template, 1> PRMS{
            PRM_Template()
    };

    static SIM_DopDescription DESC(true,
                                   "set_collider",
                                   "Set Collider",
                                   DATANAME,
                                   classname(),
                                   PRMS.data());
    setGasDescription(DESC);
    return &DESC;
}

bool GAS_SetCollider::Solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep,
                            UT_WorkBuffer &error_msg) {
    if (!obj)
    {
        error_msg.appendSprintf("Object Is Null, From %s\n", DATANAME);
        return false;
    }

    SIM_SemiAnalyticalCollider *collider = SIM_DATA_GET(*obj, SIM_SemiAnalyticalCollider::DATANAME, SIM_SemiAnalyticalCollider);
    if (!collider)
    {
        error_msg.appendSprintf("SemiAnalyticalCollider Is Null, From %s\n", DATANAME);
        return false;
    }
    if(!collider->Configured)
    {
        error_msg.appendSprintf("SemiAnalyticalCollider Not Configured Yet, From %s\n", DATANAME);
        return false;
    }

    SIM_Geometry *src_geo = SIM_DATA_GET(*this, SIM_GEOMETRY_DATANAME, SIM_Geometry);
    if (!src_geo)
    {
        error_msg.appendSprintf("Source Geometry Is Null, From %s\n", DATANAME);
        return false;
    }

    /*SIM_GeometryCopy *geo = SIM_DATA_GET(*obj, SIM_GEOMETRY_DATANAME, SIM_GeometryCopy);
    if (!geo)
    {
        error_msg.appendSprintf("Geometry Is Null, From %s\n", DATANAME);
        return false;
    }*/

    {
        SIM_GeometryAutoReadLock lock(src_geo);
        const GU_Detail *gdp_source = lock.getGdp();
        if (!gdp_source)
        {
            error_msg.appendSprintf("Source Geometry GDP is nullptr, From %s\n", DATANAME);
            return false;
        }

        collider->boundary->vertices.clear();
        collider->boundary->faces.clear();

        {
            GA_Offset pt_off;
            GA_FOR_ALL_PTOFF(gdp_source, pt_off)
                {
                    const UT_Vector3 pos = gdp_source->getPos3(pt_off);
                    collider->boundary->vertices[pt_off] = pos;
                }

            size_t triangleIndex = 0;
            const GEO_Primitive *prim;
            GA_FOR_ALL_PRIMITIVES(gdp_source, prim)
            {
                const auto *poly = dynamic_cast<const GEO_PrimPoly *>(prim);
                if (!poly)
                {
                    error_msg.appendSprintf("ERROR ON CONVERT PRIM TO POLY, From %s\n", DATANAME);
                    return false;
                }

                // Triangulate Polygon
                std::vector<size_t> polyIndices;
                for (int vi = 0; vi < poly->getVertexCount(); ++vi)
                    polyIndices.push_back(poly->getPointIndex(vi));
                for (size_t i = 1; i < polyIndices.size() - 1; ++i)
                {
                    std::vector<size_t> triangulationIndices = {polyIndices[0], polyIndices[i + 1], polyIndices[i]}; // notice the normal
                    collider->boundary->faces[triangleIndex++] = triangulationIndices;
                }

            }
        }
    }
    //collider->check_data();

    return true;
}
