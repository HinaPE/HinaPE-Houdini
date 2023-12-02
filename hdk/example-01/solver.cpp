#include "solver.h"

#include <PRM/PRM_Name.h>
#include <PRM/PRM_Template.h>
#include <PRM/PRM_Shared.h>
#include <PRM/PRM_Default.h>

#include <SIM/SIM_Engine.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_PositionSimple.h>
#include <SIM/SIM_GeometryCopy.h>

#include <GU/GU_Detail.h>
#include <GA/GA_Handle.h>

#include <array>
#include <iostream>

const SIM_DopDescription *MySolver::GetDescription()
{
	static PRM_Name my_gravity("mygravity", "My Gravity");
	static PRM_Default my_gravity_default(-9.8);

	static std::array<PRM_Template, 2> PRMS{
			PRM_Template(PRM_FLT, 1, &my_gravity, &my_gravity_default),
			PRM_Template()
	};

	static SIM_DopDescription DESC(true,
								   "my_solver",
								   "My Solver",
								   "MySolver",
								   classname(),
								   PRMS.data());
	return &DESC;
}

MySolver::MySolver(const SIM_DataFactory *factory) : SIM_Solver(factory), SIM_OptionsUser(this) {}
MySolver::~MySolver() = default;

SIM_Solver::SIM_Result MySolver::solveObjectsSubclass(SIM_Engine &engine, SIM_ObjectArray &objects, SIM_ObjectArray &newobjects, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep)
{
	for (int i = 0; i < objects.entries(); ++i)
	{
	}

	return SIM_SOLVER_SUCCESS;
}
