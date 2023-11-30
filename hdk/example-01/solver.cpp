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
	static PRM_Default my_gravity_default(9.8);

	static PRM_Name my_geometry("my_SOP", "My SOP");
	static PRM_Default my_geometry_default;

	static std::array<PRM_Template, 3> PRMS{
			PRM_Template(PRM_FLT, 1, &my_gravity, &my_gravity_default),
			PRM_Template(PRM_STRING, 1, &my_geometry, &my_geometry_default),
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
	std::cout << feedbacktoobjects.entries() << "\n";
	for (int i = 0; i < objects.entries(); ++i)
	{
		SIM_Object *obj = objects(i);

//		SIM_PositionSimple *pos;
//		pos = SIM_DATA_GET(*obj, "Position", SIM_PositionSimple);
//		if (!pos)
//			return SIM_SOLVER_FAIL;

		SIM_GeometryCopy *geo;
		geo = SIM_DATA_GET(*this, "Geometry", SIM_GeometryCopy);

		if (!geo)
			return SIM_SOLVER_FAIL;

		std::cout << geo->getNumPrimitives() << "\n";

//		{
//			SIM_GeometryAutoWriteLock lock(geo);
//			GU_Detail &gdp = lock.getGdp();
//			GA_RWHandleF vel = gdp.findAttribute(GA_ATTRIB_POINT, "velocity");
//		}

		double G = getMyGravity();
	}

	return SIM_SOLVER_SUCCESS;
}
