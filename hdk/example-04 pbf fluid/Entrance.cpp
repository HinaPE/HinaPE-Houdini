#include <UT/UT_DSOVersion.h> // Very Important!!! Include this here!!!
#include <SIM/SIM_OptionsUser.h>
#include "SIM_Interface.h"

#define SolverName "pbf_cloth_solver"
#define SolverNameEnglish "PBF Solver"
#define SolverDataName "PBFSolver"
#define SolverDescription "PBF Solver Description"


#define NewSolverParameterGetSet \
GETSET_DATA_FUNCS_V3("gravity", Gravity);

ImplementNewSolverHeader(SIM_PBFSolver)

ImplementNewSolverImpl(SIM_PBFSolver)

void initializeSIM(void *)
{
	IMPLEMENT_DATAFACTORY(SIM_PBFSolver)
}

const SIM_DopDescription *SIM_PBFSolver::GetDescription()
{
	static PRM_Name gravity("gravity", "Gravity");
	static std::array<PRM_Default, 3> gravity_default = {0, -9.8, 0};
	static std::array<PRM_Template, 4> PRMS{
			PRM_Template(PRM_FLT_J, 3, &gravity, gravity_default.data()),
			PRM_Template()
	};
	static SIM_DopDescription DESC(true, SolverName, SolverNameEnglish, SolverDataName, classname(), PRMS.data());
	return &DESC;
}

SIM_Solver::SIM_Result SIM_PBFSolver::solveSingleObjectSubclass(SIM_Engine &, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep, bool newobject)
{
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
