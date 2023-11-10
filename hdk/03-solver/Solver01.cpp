#include "Solver01.h"

#include <UT/UT_DSOVersion.h>
#include <PRM/PRM_Include.h>
#include <SIM/SIM_PRMShared.h>
#include <SIM/SIM_DopDescription.h>

#include <SIM/SIM_Object.h>
#include <SIM/SIM_Position.h>

#include <iostream>

void
initializeSIM(void *)
{
	IMPLEMENT_DATAFACTORY(Solver01);
}

Solver01::Solver01(const SIM_DataFactory *factory) : SIM_SingleSolver(factory), SIM_OptionsUser(this) {}
Solver01::~Solver01() = default;

auto Solver01::getSolver01Description() -> const SIM_DopDescription *
{
	static PRM_Name theTest("test", "Test");

	static PRM_Template theTemplates[] = {
			PRM_Template(PRM_FLT_J,	1, &theTest, PRMoneDefaults),
			PRM_Template()
	};

	static SIM_DopDescription theDopDescription(true,
												"hina_solver01",	// operator name
												"Solver 01",	// English name
												"Solver01",		// default data name
												classname(),
												theTemplates);

	return &theDopDescription;
}
auto Solver01::solveSingleObjectSubclass(SIM_Engine &engine, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep, bool newobject) -> SIM_Solver::SIM_Result
{
	// init
	if (newobject)
	{

	}

//	UT_Vector3 pos;
//	object.getPosition()->getPosition(pos);
//	std::cout << pos << "\n";

//	// update
//	const auto& name = object.getName();
//	std::cout << name << '\n';
	auto *geo = object.getGeometry();
//
//	{
//		SIM_GeometryAutoReadLock lock(geo);
//		auto& gdp = lock.getGdp();
//
//		GA_Index;
//	}
//
	return geo ? SIM_SOLVER_SUCCESS : SIM_SOLVER_FAIL;
}

void MyGeometry::initializeSubclass()
{
	SIM_Geometry::initializeSubclass();
}
auto MyGeometry::getGeometrySubclass() const -> GU_ConstDetailHandle
{
	return SIM_Geometry::getGeometrySubclass();
}
