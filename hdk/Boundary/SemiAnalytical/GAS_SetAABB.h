//
// Created by LiYifan on 2024/2/29.
//

#ifndef HINAPE_HOUDINI_GAS_SETAABB_H
#define HINAPE_HOUDINI_GAS_SETAABB_H

#include <GAS/GAS_SubSolver.h>
#include <PRM/PRM_Default.h>

class GAS_SetAABB : public GAS_SubSolver {
public:
    static const char *DATANAME;
protected:
    GAS_SetAABB(const SIM_DataFactory *factory) : BaseClass(factory) {}
    ~GAS_SetAABB() override = default;
    bool solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) override;
    void initializeSubclass() override;
    void makeEqualSubclass(const SIM_Data *source) override;
    static const SIM_DopDescription *getDopDescription();

DECLARE_STANDARD_GETCASTTOTYPE();
DECLARE_DATAFACTORY(GAS_SetAABB,
                    GAS_SubSolver,
                    "SetAABB",
                    getDopDescription());

private:
    bool Solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep, UT_WorkBuffer &error_msg) const;
};
#endif //HINAPE_HOUDINI_GAS_SETAABB_H
