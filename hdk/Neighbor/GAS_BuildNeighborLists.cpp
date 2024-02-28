//
// Created by LiYifan on 2024/2/28.
//

#include "GAS_BuildNeighborLists.h"
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

bool GAS_BuildNeighborLists::solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) {
    UT_WorkBuffer error_msg;
    if (!Solve(engine, obj, time, timestep, error_msg) || UTisstring(error_msg.buffer()))
    {
        SIM_Data::addError(obj, SIM_MESSAGE, error_msg.buffer(), UT_ERROR_ABORT);
        return false;
    }

    return true;
}

void GAS_BuildNeighborLists::initializeSubclass() {
    SIM_Data::initializeSubclass();

    this->nsearch = nullptr;
    this->cached_positions.clear();
    this->cached_point_set_indices.clear();
}

void GAS_BuildNeighborLists::makeEqualSubclass(const SIM_Data *source) {
    SIM_Data::makeEqualSubclass(source);

    const GAS_BuildNeighborLists *src = SIM_DATA_CASTCONST(source, GAS_BuildNeighborLists);
    this->nsearch = nullptr;
    this->cached_positions = src->cached_positions;
    this->cached_point_set_indices = src->cached_point_set_indices;
}

const char *GAS_BuildNeighborLists::DATANAME = "BuildNeighborLists";
const SIM_DopDescription *GAS_BuildNeighborLists::getDopDescription()
{
    static PRM_Name maxNumberOfNeighbors("MaxNumberOfNeighbors", "Max Number Of Neighbors");
    static PRM_Default maxNumberOfNeighborsDefault(0);

    static PRM_Name kernelRadius("KernelRadius", "Kernel Radius");
    static PRM_Default kernelRadiusDefault(0.036);

    static std::array<PRM_Template, 3> PRMS{
            PRM_Template(PRM_INT, 1, &maxNumberOfNeighbors, &maxNumberOfNeighborsDefault),
            PRM_Template(PRM_FLT, 1, &kernelRadius, &kernelRadiusDefault),
            PRM_Template()
    };

    static SIM_DopDescription DESC(true,
                                   "build_neighbor_lists",
                                   "BuildNeighborLists",
                                   DATANAME,
                                   classname(),
                                   PRMS.data());
    setGasDescription(DESC);
    return &DESC;
}


bool GAS_BuildNeighborLists::Solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep,
                                   UT_WorkBuffer &error_msg){
    if (!obj)
    {
        error_msg.appendSprintf("Object Is Null, From %s\n", DATANAME);
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

    /*long long maxNumberOfNeighbors = getMaxNumberOfNeighbors();
    if(maxNumberOfNeighbors <= 0) {
        // request dynamic neighbor list

    }else{
        // request fixed neighbor list
    }*/

    if(!nsearch)
    {
        nsearch = new cuNSearch::NeighborhoodSearch(getKernelRadius());
        cached_positions.clear();
        cached_point_set_indices.clear();

        // ========== 1. Add Fluid Particles ==========
        std::vector<std::array<cuNSearch::Real, 3>> &positions = cached_positions["Fluid"];

        {
            positions.resize(particleData->pbfParticle->positions.size());
            particleData->for_each_offset([&](const GA_Offset &pt_off)
            {
                GA_Index pt_idx = particleData->offset2index[pt_off];
                UT_Vector3 pos = particleData->pbfParticle->positions[pt_off];
                positions[pt_idx][0] = pos.x();
                positions[pt_idx][1] = pos.y();
                positions[pt_idx][2] = pos.z();
            });
        }
        cached_point_set_indices["Fluid"] = nsearch->add_point_set(positions.front().data(), positions.size(), true /* dynamic */, true /* search neighbors */, true /* BE SEARCHED by other points sets*/);

        // ========== 3. Search ==========
        nsearch->set_active(true); // for first search, we search for all other point sets with all other point sets
        nsearch->find_neighbors();

        // ========== 4. Update Neighbor Cache ==========
        particleData->pbfParticle->point_neighbors.clear();
        const unsigned int point_set_index = cached_point_set_indices["Fluid"];
        auto &point_set = nsearch->point_set(point_set_index);

        particleData->for_each_offset([&](const GA_Offset &pt_off)
        {
            GA_Index pt_idx = particleData->offset2index[pt_off];
            for (const auto &pair: cached_point_set_indices) // 其实只有一个类型现在
            {
                const UT_String &other_point_set_name = pair.first;
                const unsigned int other_point_set_index = pair.second;
                auto neighbor_count = point_set.n_neighbors(other_point_set_index, pt_idx);
                for (int nidx = 0; nidx < neighbor_count; ++nidx) {
                    const auto n_idx = point_set.neighbor(other_point_set_index, pt_idx, nidx);

                    if (other_point_set_index == point_set_index) // self neighbors
                    {
                        const GA_Offset n_off = particleData->index2offset[n_idx];
                        const UT_Vector3 n_pos = UT_Vector3D{
                                point_set.GetPoints()[3 * n_idx + 0],
                                point_set.GetPoints()[3 * n_idx + 1],
                                point_set.GetPoints()[3 * n_idx + 2]};
                        ParticleState ns{};
                        ns.pt_off = n_off;
                        ns.pt_pos = n_pos;
                        particleData->pbfParticle->point_neighbors[pt_off].emplace_back(ns);
                    }
                }
            }
        });

        nsearch->set_active(cached_point_set_indices["Fluid"], false, true);
        // particleData->check_data();
    }else{
        std::vector<std::array<cuNSearch::Real, 3>> &positions = cached_positions["Fluid"];

        {
            positions.resize(particleData->pbfParticle->positions.size());
            particleData->for_each_offset([&](const GA_Offset &pt_off)
                                          {
                                              GA_Index pt_idx = particleData->offset2index[pt_off];
                                              UT_Vector3 pos = particleData->pbfParticle->positions[pt_off];
                                              positions[pt_idx][0] = pos.x();
                                              positions[pt_idx][1] = pos.y();
                                              positions[pt_idx][2] = pos.z();
                                          });
        }
        nsearch->resize_point_set(cached_point_set_indices["Fluid"], positions.front().data(), positions.size());
        nsearch->update_point_set(cached_point_set_indices["Fluid"]);

        nsearch->find_neighbors();

        particleData->pbfParticle->point_neighbors.clear();
        const unsigned int point_set_index = cached_point_set_indices["Fluid"];
        auto &point_set = nsearch->point_set(point_set_index);

        particleData->for_each_offset([&](const GA_Offset &pt_off)
                                      {
                                          GA_Index pt_idx = particleData->offset2index[pt_off];
                                          for (const auto &pair: cached_point_set_indices) // 其实只有一个类型现在
                                          {
                                              const UT_String &other_point_set_name = pair.first;
                                              const unsigned int other_point_set_index = pair.second;
                                              auto neighbor_count = point_set.n_neighbors(other_point_set_index, pt_idx);
                                              for (int nidx = 0; nidx < neighbor_count; ++nidx) {
                                                  const auto n_idx = point_set.neighbor(other_point_set_index, pt_idx, nidx);

                                                  if (other_point_set_index == point_set_index) // self neighbors
                                                  {
                                                      const GA_Offset n_off = particleData->index2offset[n_idx];
                                                      const UT_Vector3 n_pos = UT_Vector3D{
                                                              point_set.GetPoints()[3 * n_idx + 0],
                                                              point_set.GetPoints()[3 * n_idx + 1],
                                                              point_set.GetPoints()[3 * n_idx + 2]};
                                                      ParticleState ns{};
                                                      ns.pt_off = n_off;
                                                      ns.pt_pos = n_pos;
                                                      particleData->pbfParticle->point_neighbors[pt_off].emplace_back(ns);
                                                  }
                                              }
                                          }
                                      });

        nsearch->set_active(cached_point_set_indices["Fluid"], false, true);
    }

    particleData->commit(geo);
    return true;
}
