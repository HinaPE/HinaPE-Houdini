//
// Created by LiYifan on 2024/2/27.
//

#ifndef HINAPE_HOUDINI_GAS_VOLUMEPARTICLEEMITTER_H
#define HINAPE_HOUDINI_GAS_VOLUMEPARTICLEEMITTER_H

#include <GAS/GAS_SubSolver.h>
#include <PRM/PRM_Default.h>

#include "Core/Emitter/VolumeParticleEmitter3.hpp"

class GAS_VolumeParticleEmitter : public GAS_SubSolver
{
public:
    static const char *DATANAME;
    /*CubbyFlow::VolumeParticleEmitter3Ptr InnerPtr;*/

    bool Emitted{};
    int EmittedParticles{};

    GETSET_DATA_FUNCS_F("Jitter", Jitter)
    GETSET_DATA_FUNCS_I("MaxNumberOfParticles", MaxNumberOfParticles)
    GETSET_DATA_FUNCS_B("IsOneShot", IsOneShot)
    GETSET_DATA_FUNCS_B("IsEmitterMove", IsEmitterMove)
    GETSET_DATA_FUNCS_I("RandomSeed", RandomSeed)

protected:
    explicit GAS_VolumeParticleEmitter(const SIM_DataFactory *factory) : BaseClass(factory) {}
    ~GAS_VolumeParticleEmitter() override = default;
    bool solveGasSubclass(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep) override;
    void initializeSubclass() override;
    void makeEqualSubclass(const SIM_Data *source) override;
    static const SIM_DopDescription *getDopDescription();

    DECLARE_STANDARD_GETCASTTOTYPE();
    DECLARE_DATAFACTORY(GAS_VolumeParticleEmitter,
                        GAS_SubSolver,
                        "VolumeParticleEmitter SubSolver",
                        getDopDescription());
private:
    //bool InitRuntime(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep, UT_WorkBuffer &error_msg);
    bool Solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep, UT_WorkBuffer &error_msg);
};

#endif //HINAPE_HOUDINI_GAS_VOLUMEPARTICLEEMITTER_H
