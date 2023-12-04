#include <UT/UT_DSOVersion.h> // Very Important!!! Include this here!!!
#include "solver.h"

#define SolverName "pbf_solver"
#define SolverNameEnglish "PBF Solver"
#define SolverDataName "PBFSolver"
#define SolverDescription "PBF Solver Description"

#define NewSolverParameterGetSet \
GETSET_DATA_FUNCS_F("dt", Dt); \
GETSET_DATA_FUNCS_F("particle_radius", ParticleRadius); \
GETSET_DATA_FUNCS_F("kernel_radius", KernelRadius); \
GETSET_DATA_FUNCS_F("target_density", TargetDensity); \
GETSET_DATA_FUNCS_F("viscosity", Viscosity); \
GETSET_DATA_FUNCS_F("vorticity", Vorticity); \
GETSET_DATA_FUNCS_V3("gravity", Gravity); \


ImplementNewSolverHeader(SIM_PBFSolver)

ImplementNewSolverImpl(SIM_PBFSolver)

ImplementInnerSolverRequired(HinaPE::PBFSolver)

void initializeSIM(void *)
{
	IMPLEMENT_DATAFACTORY(SIM_PBFSolver)
}

const SIM_DopDescription *SIM_PBFSolver::GetDescription()
{
	static PRM_Name dt("dt", "Dt");
	static PRM_Default dt_default(0.005);

	static PRM_Name particle_radius("particle_radius", "Particle Radius");
	static PRM_Default particle_radius_default(0.029);

	static PRM_Name kernel_radius("kernel_radius", "Kernel Radius");
	static PRM_Default kernel_radius_default = particle_radius_default.getFloat() * 1.7;

	static PRM_Name target_density("target_density", "Target Density");
	static PRM_Default target_density_default(1000);

	static PRM_Name viscosity("viscosity", "Viscosity");
	static PRM_Default viscosity_default(0.1);

	static PRM_Name vorticity("vorticity", "Vorticity");
	static PRM_Default vorticity_default(0.00001);

	static PRM_Name gravity("gravity", "Gravity");
	static std::array<PRM_Default, 3> gravity_default = {0, -9.8, 0};

	static std::array<PRM_Template, 8> PRMS{
			PRM_Template(PRM_FLT_J, 1, &dt, &dt_default),
			PRM_Template(PRM_FLT_J, 1, &particle_radius, &particle_radius_default),
			PRM_Template(PRM_FLT_J, 1, &kernel_radius, &kernel_radius_default),
			PRM_Template(PRM_FLT_J, 1, &target_density, &target_density_default),
			PRM_Template(PRM_FLT_J, 1, &viscosity, &viscosity_default),
			PRM_Template(PRM_FLT_J, 1, &vorticity, &vorticity_default),
			PRM_Template(PRM_FLT_J, 3, &gravity, gravity_default.data()),
			PRM_Template()
	};
	static SIM_DopDescription DESC(true, SolverName, SolverNameEnglish, SolverDataName, classname(), PRMS.data());
	return &DESC;
}

SIM_Solver::SIM_Result SIM_PBFSolver::solveSingleObjectSubclass(SIM_Engine &, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep, bool newobject)
{
	params.RealParams["dt"] = getDt();
	params.RealParams["particle_radius"] = getParticleRadius();
	params.RealParams["kernel_radius"] = getKernelRadius();
	params.RealParams["target_density"] = getTargetDensity();
	params.RealParams["viscosity"] = getViscosity();
	params.RealParams["vorticity"] = getVorticity();
	params.Vector3Params["gravity"] = getGravity();
	ISolver::Param = params;

	static bool NeedReBuild = true;
	auto *geo = GetFromSubData<SIM_GeometryCopy>(object, "Geometry");
	auto *pos = GetFromSubData<SIM_PositionSimple>(object, "Position");
	if (NeedReBuild || newobject)
	{
		ISolver::Solver->InitGeometry(geo);
		ISolver::Solver->InitPosition(pos);
		NeedReBuild = false;
	} else
	{
		ISolver::Solver->SolveGeometry(geo, timestep);
		ISolver::Solver->SolvePosition(pos, timestep);
	}
	return SIM_Solver::SIM_SOLVER_SUCCESS;
}
