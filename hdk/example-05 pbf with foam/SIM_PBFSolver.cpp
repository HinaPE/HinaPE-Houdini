#include "SIM_PBFSolver.h"

#include "pbf/solver.h"

#include <PRM/PRM_Name.h>
#include <PRM/PRM_Template.h>
#include <PRM/PRM_Shared.h>
#include <PRM/PRM_Default.h>

#include <SIM/SIM_Engine.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_Collider.h>
#include <SIM/SIM_ColliderPoint.h>
#include <SIM/SIM_RelationshipSource.h>

#include <GU/GU_Detail.h>
#include <GA/GA_Handle.h>
#include <GA/GA_Primitive.h>

#include <UT/UT_ThreadedAlgorithm.h>
#include <UT/UT_ParallelUtil.h>
#include <UT/UT_ParallelPipeline.h>

#include <array>
#include <iostream>

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
//	static bool NeedReBuild = true;
//
//	if (NeedReBuild || newobject)
//	{
////		init(object);
//		NeedReBuild = false;
//	} else
//	{
////		solve(object, timestep);
//	}

	SIM_GeometryCopy *geo = SIM_DATA_GET(object, "Geometry", SIM_GeometryCopy);
	SIM_GeometryAutoWriteLock lock(geo);
	GU_Detail& gdp = lock.getGdp();

	std::cout << "obj points offset: ";
	for (GA_Size i = 0; i < gdp.getNumPoints(); ++i)
	{
		GA_Offset offset = gdp.pointOffset(i);
		std::cout << offset << " ";
	}
	std::cout << "\n";

	// After Effect
	SIM_ObjectArray affectors;
	object.getAffectors(affectors, "SIM_RelationshipCollide");
	for (GA_Size i = 0; i < affectors.entries(); ++i)
	{
		SIM_Object &affector = *affectors(i);
		if (!affector.getName().equal(object.getName()))
		{
			SIM_Geometry *collider_geo = SIM_DATA_GET(affector, "Geometry", SIM_Geometry);
			SIM_GeometryAutoReadLock lock(collider_geo);
			const GU_Detail* collider_gdp = lock.getGdp();

			std::cout << "collider points offset: ";
			for (GA_Size i = 0; i < collider_gdp->getNumPoints(); ++i)
			{
				GA_Offset offset = collider_gdp->pointOffset(i);
				std::cout << offset << " ";
			}
			std::cout << "\n";

			GU_Detail *merged_gdp = new GU_Detail();
			merged_gdp->merge(gdp);
			merged_gdp->merge(*collider_gdp);

			std::cout << "merged points offset: ";
			for (GA_Size i = 0; i < merged_gdp->getNumPoints(); ++i)
			{
				GA_Offset offset = merged_gdp->pointOffset(i);
				std::cout << offset << " ";
			}
			std::cout << "\n";
		}
	}
	return SIM_Solver::SIM_SOLVER_SUCCESS;
}
