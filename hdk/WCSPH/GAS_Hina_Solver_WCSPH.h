//
// Created by LiYifan on 2024/4/15.
//

#ifndef HINAPE_HOUDINI_GAS_HINA_SOLVER_WCSPH_H
#define HINAPE_HOUDINI_GAS_HINA_SOLVER_WCSPH_H

#include <SIM/SIM_SingleSolver.h>
#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Utils.h>

class SPHSolver : public SIM_SingleSolver, public SIM_OptionsUser
{
public:
    GETSET_DATA_FUNCS_V3("volume_min", VolumeMin);
    GETSET_DATA_FUNCS_V3("volume_max", VolumeMax);
    GETSET_DATA_FUNCS_F("mass", Mass);
    GETSET_DATA_FUNCS_F("rest_density", RestDensity);
    GETSET_DATA_FUNCS_F("pressure_constant", PressureConstant);
    GETSET_DATA_FUNCS_F("time_step", TimeStep);
    GETSET_DATA_FUNCS_F("kernel_radius", KernelRadius);

    GETSET_DATA_FUNCS_F("bulk_viscosity", BulkViscosity);
    GETSET_DATA_FUNCS_F("shear_viscosity", ShearViscosity);
private:
    explicit SPHSolver(const SIM_DataFactory *factory) : SIM_SingleSolver(factory), SIM_OptionsUser(this) {}
    ~SPHSolver() override = default;
    SIM_Result solveSingleObjectSubclass(SIM_Engine &engine, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep, bool newobject) override;
    static const SIM_DopDescription *GetDescription();

DECLARE_STANDARD_GETCASTTOTYPE();
DECLARE_DATAFACTORY(SPHSolver, SIM_SingleSolver, "SPH Solver Description", GetDescription());

    // ==================== Custom Field ====================
protected:
    void init(SIM_Object &obj);
    void solve(SIM_Object &obj, const SIM_Time &dt);

    void run(fpreal time, SIM_Object &obj);

    void precomputeKernelCoefficients() const;

    static fpreal	_halfH;
    static fpreal	_kernelValueCoeff;
    static fpreal	_kernelGradientCoeffA;
    static fpreal	_kernelGradientCoeffB;

    fpreal getKernelValue(fpreal dist) const;
    UT_Vector3 getKernelGradient(fpreal dist, const UT_Vector3 &xij) const;

    struct Neighbor
    {
        Neighbor(long i, const UT_Vector3& x, double d);

        long	id;
        fpreal	dist;
        UT_Vector3	xij{};
    };
};

/// A simple Logger
#include <iostream>
#include <vector>
#include <string>

static struct WCSPHLogger
{
    SIM_Solver::SIM_Result report()
    {
        for (const auto &l: log) std::cout << l << '\n';
        return log.empty() ? SIM_Solver::SIM_SOLVER_SUCCESS : SIM_Solver::SIM_SOLVER_FAIL;
    }
    void reset() { log.clear(); }
    void error_nullptr(const std::string &type) { log.emplace_back("NullPointerError::" + type); }

    std::vector<std::string> log;
} WCSPHLog;

#endif //HINAPE_HOUDINI_GAS_HINA_SOLVER_WCSPH_H
