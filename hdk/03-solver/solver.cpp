#include "solver.h"

#include <PRM/PRM_Include.h>
#include <SIM/SIM_PRMShared.h>
#include <SIM/SIM_DopDescription.h>

#include <SIM/SIM_Object.h>
#include <SIM/SIM_Position.h>
#include <SIM/SIM_GeometryCopy.h>

#include <iostream>

HinaClothSolver::HinaClothSolver(const SIM_DataFactory *factory) : SIM_SingleSolver(factory), SIM_OptionsUser(this) {}
HinaClothSolver::~HinaClothSolver() = default;

auto HinaClothSolver::getSolver01Description() -> const SIM_DopDescription *
{
	static PRM_Name theTest("test", "Test");

	static std::array<PRM_Template, 2> PRMS{
			PRM_Template(PRM_FLT_J,	1, &theTest, PRMoneDefaults),
			PRM_Template()
	};

	static SIM_DopDescription theDopDescription(true,
												"hina_cloth_solver",	// operator name
												"Hina Cloth Solver",	// English name
												"HinaClothSolver",	// default data name
												classname(),
												PRMS.data());

	return &theDopDescription;
}
auto HinaClothSolver::solveSingleObjectSubclass(SIM_Engine &engine, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep, bool newobject) -> SIM_Solver::SIM_Result
{
	SIM_GeometryCopy *geo = nullptr;
	// init
	if (newobject)
	{
		geo = SIM_DATA_CREATE(object, "Geometry", SIM_GeometryCopy, SIM_DATA_RETURN_EXISTING | SIM_DATA_ADOPT_EXISTING_ON_DELETE);

		UT_String name = object.getName();
		std::cout << name << "\n";
	}
	else
	{
//		geo = object.getGeometry();
		geo = SIM_DATA_GET(object, "Geometry", SIM_GeometryCopy);

		UT_String name = object.getName();
		std::cout << name << "\n";
	}
	return geo ? SIM_SOLVER_SUCCESS : SIM_SOLVER_FAIL;
}
