#include "Solver01.h"

#include <UT/UT_DSOVersion.h>
#include <PRM/PRM_Include.h>
#include <SIM/SIM_PRMShared.h>
#include <SIM/SIM_DopDescription.h>

auto Solver01::solveObjectsSubclass(SIM_Engine &engine, SIM_ObjectArray &objects, SIM_ObjectArray &newobjects, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep) -> SIM_Solver::SIM_Result
{
	return SIM_SOLVER_SUCCESS;
}

Solver01::Solver01(const SIM_DataFactory *factory) : SIM_Solver(factory), SIM_OptionsUser(this) {}
Solver01::~Solver01() = default;

auto Solver01::getSolver01Description() -> const SIM_DopDescription *
{
	static PRM_Name theTest("test", "Test");

	static PRM_Template theTemplates[] = {
			PRM_Template(PRM_FLT_J,	1, &theTest, PRMoneDefaults),
			PRM_Template()
	};

	static SIM_DopDescription theDopDescription(true,
												"hina_sim01",	// operator name
												"SIM 01",	// English name
												"SIM01",		// default data name
												classname(),
												theTemplates);

	return &theDopDescription;
}
