#include "solver.h"

#include <PRM/PRM_Include.h>
#include <SIM/SIM_PRMShared.h>
#include <SIM/SIM_DopDescription.h>

#include <SIM/SIM_Object.h>
#include <SIM/SIM_Position.h>
#include <SIM/SIM_GeometryCopy.h>

#include <GU/GU_Detail.h>
#include <GEO/GEO_PrimList.h>
#include <GA/GA_Types.h>

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

	if (newobject)
	{
		geo = SIM_DATA_CREATE(object, "Geometry", SIM_GeometryCopy, SIM_DATA_RETURN_EXISTING | SIM_DATA_ADOPT_EXISTING_ON_DELETE);
	}
	else
	{
		geo = SIM_DATA_GET(object, "Geometry", SIM_GeometryCopy);
	}

	if (!geo)
		return SIM_SOLVER_FAIL;

	{
		SIM_GeometryAutoWriteLock write_lock(geo);
		GU_Detail& gdp = write_lock.getGdp();

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
