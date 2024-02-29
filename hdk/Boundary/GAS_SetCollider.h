//
// Created by LiYifan on 2024/2/29.
//

#ifndef HINAPE_HOUDINI_GAS_SETCOLLIDER_H
#define HINAPE_HOUDINI_GAS_SETCOLLIDER_H

#include <GAS/GAS_SubSolver.h>
#include <PRM/PRM_Default.h>

class GAS_SetCollider : public GAS_SubSolver
{
public:
    static const char *DATANAME;
    explicit GAS_SetCollider(const SIM_DataFactory *factory) : BaseClass(factory) {}
    ~GAS_SetCollider() override = default;
    bool solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) override;
    void initializeSubclass() override;
    void makeEqualSubclass(const SIM_Data *source) override;
    static const SIM_DopDescription *getDopDescription();

    DECLARE_STANDARD_GETCASTTOTYPE();
    DECLARE_DATAFACTORY(GAS_SetCollider,
                        GAS_SubSolver,
                        "Set_Collider",
                        getDopDescription());
private:
    bool Solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep, UT_WorkBuffer &error_msg);
};

#endif //HINAPE_HOUDINI_GAS_SETCOLLIDER_H
