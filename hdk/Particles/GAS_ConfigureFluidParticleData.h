//
// Created by LiYifan on 2024/2/27.
//

#ifndef HINAPE_HOUDINI_GAS_CONFIGUREFLUIDPARTICLEDATA_H
#define HINAPE_HOUDINI_GAS_CONFIGUREFLUIDPARTICLEDATA_H

#include <GAS/GAS_SubSolver.h>

class GAS_ConfigureFluidParticleData : public GAS_SubSolver
{
public:
    static const char *DATANAME;
    explicit GAS_ConfigureFluidParticleData(const SIM_DataFactory *factory) : BaseClass(factory) {}
    ~GAS_ConfigureFluidParticleData() override = default;
    bool solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) override;
    void initializeSubclass() override;
    void makeEqualSubclass(const SIM_Data *source) override;
    static const SIM_DopDescription *getDopDescription();

    DECLARE_STANDARD_GETCASTTOTYPE();
    DECLARE_DATAFACTORY(GAS_ConfigureFluidParticleData,
                        GAS_SubSolver,
                        "Configure_Fluid_Particle_Data",
                        getDopDescription());
private:
    static bool Solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep, UT_WorkBuffer &error_msg);
};

#endif //HINAPE_HOUDINI_GAS_CONFIGUREFLUIDPARTICLEDATA_H
