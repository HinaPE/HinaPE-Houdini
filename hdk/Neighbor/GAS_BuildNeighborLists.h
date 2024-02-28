//
// Created by LiYifan on 2024/2/28.
//

#ifndef HINAPE_HOUDINI_GAS_BUILDNEIGHBORLISTS_H
#define HINAPE_HOUDINI_GAS_BUILDNEIGHBORLISTS_H

#include <GAS/GAS_SubSolver.h>
#include <PRM/PRM_Default.h>

#include "Searcher/PointNeighborSearcher.hpp"
#include "cuNSearch.h"

class GAS_BuildNeighborLists : public GAS_SubSolver
{
public:
    static const char *DATANAME;
    GETSET_DATA_FUNCS_I("MaxNumberOfNeighbors", MaxNumberOfNeighbors)
    GETSET_DATA_FUNCS_F("KernelRadius", KernelRadius)

protected:
    explicit GAS_BuildNeighborLists(const SIM_DataFactory *factory) : BaseClass(factory) {}
    ~GAS_BuildNeighborLists() override = default;
    bool solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) override;
    void initializeSubclass() override;
    void makeEqualSubclass(const SIM_Data *source) override;
    static const SIM_DopDescription *getDopDescription();

DECLARE_STANDARD_GETCASTTOTYPE();
DECLARE_DATAFACTORY(GAS_BuildNeighborLists,
                    GAS_SubSolver,
                    "BuildNeighborLists",
                    getDopDescription());

private:
    bool Solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep, UT_WorkBuffer &error_msg);

    cuNSearch::NeighborhoodSearch* nsearch;
    std::map<UT_String, std::vector<std::array<fpreal, 3>>> cached_positions;
    std::map<UT_String, unsigned int> cached_point_set_indices;
};
#endif //HINAPE_HOUDINI_GAS_BUILDNEIGHBORLISTS_H
