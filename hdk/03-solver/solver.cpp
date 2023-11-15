#include "solver.h"

#include <PRM/PRM_Include.h>
#include <SIM/SIM_PRMShared.h>
#include <SIM/SIM_DopDescription.h>

#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_Position.h>
#include <SIM/SIM_PositionSimple.h>
#include <SIM/SIM_PositionComposite.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_ColliderInfo.h>


#include <GU/GU_Detail.h>
#include <GEO/GEO_PrimList.h>
#include <GA/GA_Types.h>

#include <iostream>

HinaClothSolver::HinaClothSolver(const SIM_DataFactory *factory) : SIM_SingleSolver(factory), SIM_OptionsUser(this) {}
HinaClothSolver::~HinaClothSolver() = default;

auto HinaClothSolver::solveSingleObjectSubclass(SIM_Engine &engine, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep, bool newobject) -> SIM_Solver::SIM_Result
{
	// We don't deal with newobject occasion currently
	if (newobject)
		return SIM_SOLVER_FAIL;

	SIM_GeometryCopy *geo = nullptr;
	SIM_Position *pos = nullptr;

	geo = SIM_DATA_GET(object, "Geometry", SIM_GeometryCopy);
	pos = SIM_DATA_GET(object, "Position", SIM_Position);

	if (!geo || !pos)
		return SIM_SOLVER_FAIL;

	{

	}




	{
		SIM_GeometryAutoWriteLock write_lock(geo);
		GU_Detail& gdp = write_lock.getGdp();

		object.getPosition();

		GEO_PrimList prims = gdp.primitives();
		GA_Size num_prims = prims.entries();

		GA_Size num_points = gdp.getNumPoints();
		for (GA_Size i = 0; i < num_points; ++i)
		{
			GA_Offset offset = gdp.pointOffset(i);
			UT_Vector3 pos = gdp.getPos3(offset);

			GA_ROHandleF density  = gdp.findAttribute(GA_ATTRIB_POINT, "density");
			if (density.isValid())
			{
				std::cout << "density: " << density.get(offset) << '\n';
			}
		}
	}

	return SIM_SOLVER_SUCCESS;
}
auto HinaClothSolver::getDescription() -> const SIM_DopDescription *
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

HinaClothSolver2::HinaClothSolver2(const SIM_DataFactory *factory) : SIM_Solver(factory), SIM_OptionsUser(this) {}
HinaClothSolver2::~HinaClothSolver2() = default;

auto HinaClothSolver2::solveObjectsSubclass(SIM_Engine &engine, SIM_ObjectArray &objects, SIM_ObjectArray &newobjects, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep) -> SIM_Solver::SIM_Result
{
	std::cout << "feedback: " << feedbacktoobjects.entries() << '\n';
	for (int i = 0; i < objects.entries(); ++i)
	{
		SIM_Object *object = objects(i);
		SIM_PositionSimple *pos = nullptr;

		pos = SIM_DATA_GET(*object, "Position", SIM_PositionSimple);

		if (!pos)
			return SIM_SOLVER_FAIL;

		const fpreal64 G = getGravity();
		UT_Vector3 p = pos->getPosition();
		p.y() -= G;
		pos->setPosition(p);
	}
	return SIM_SOLVER_SUCCESS;
}

auto HinaClothSolver2::getDescription() -> const SIM_DopDescription *
{
	static PRM_Name theGravity("gravity", "Gravity");

	static std::array<PRM_Template, 2> PRMS{
			PRM_Template(PRM_FLT_J,	1, &theGravity, PRMoneDefaults),
			PRM_Template()
	};

	static SIM_DopDescription theDopDescription(true,
												"hina_cloth_solver2",	// operator name
												"Hina Cloth Solver 2",	// English name
												"HinaClothSolver2",	// default data name
												classname(),
												PRMS.data());

	return &theDopDescription;
}
