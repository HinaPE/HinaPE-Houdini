#include "SIM_PBFSolver.h"

#include <PRM/PRM_Name.h>
#include <PRM/PRM_Template.h>
#include <PRM/PRM_Shared.h>
#include <PRM/PRM_Default.h>

#include <SIM/SIM_Engine.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_GeometryCopy.h>

#include <GU/GU_Detail.h>
#include <GA/GA_Handle.h>
#include <GA/GA_Primitive.h>

#include <UT/UT_ThreadedAlgorithm.h>
#include <UT/UT_ParallelUtil.h>
#include <UT/UT_ParallelPipeline.h>

#include <array>
#include <iostream>

UT_StringHolder EdgeClothGroupName("EdgesForCloth");
UT_StringHolder IsFixAttributeName("IsFix");
UT_StringHolder PredictPositionAttributeName("PdP");

const SIM_DopDescription *SIM_PBFSolver::GetDescription()
{
	static PRM_Name gravity("gravity", "Gravity");
	static std::array<PRM_Default, 3> gravity_default = {0, -9.8, 0};

	static PRM_Name stiffness("stiffness", "Stiffness");
	static PRM_Default stiffness_default(1);

	static PRM_Name constraint_iteration("constraint_iteration", "Constraint Iteration");
	static PRM_Default constraint_iteration_default(3);

	static std::array<PRM_Template, 4> PRMS{
			PRM_Template(PRM_FLT_J, 3, &gravity, gravity_default.data()),
			PRM_Template(PRM_FLT_J, 1, &stiffness, &stiffness_default),
			PRM_Template(PRM_INT_J, 1, &constraint_iteration, &constraint_iteration_default),
			PRM_Template()
	};

	static SIM_DopDescription DESC(true,
								   "pbf_solver",
								   "PBF Solver",
								   "PBFSolver",
								   classname(),
								   PRMS.data());
	return &DESC;
}

SIM_Solver::SIM_Result SIM_PBFSolver::solveSingleObjectSubclass(SIM_Engine &, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep, bool newobject)
{
	static bool NeedReBuild = true;

	if (NeedReBuild || newobject)
	{
//		init(object);
		NeedReBuild = false;
	} else
	{
//		solve(object, timestep);
	}

	return SIM_Solver::SIM_SOLVER_SUCCESS;
}
