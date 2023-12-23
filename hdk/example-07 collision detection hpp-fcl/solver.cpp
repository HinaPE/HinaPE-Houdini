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

#include <filesystem>
#include <iostream>

#include "lib1.h"

const SIM_DopDescription *HPPFCLCollisionSolver::GetDescription()
{
	static std::array<PRM_Template, 1> PRMS{
			PRM_Template()
	};

	static SIM_DopDescription DESC(true,
								   "hpp_fcl_collision_solver",
								   "HPP-FCL Collision Solver",
								   "HPPFCLCollisionSolver",
								   classname(),
								   PRMS.data());
	return &DESC;
}

SIM_Solver::SIM_Result HPPFCLCollisionSolver::solveSingleObjectSubclass(SIM_Engine &, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep, bool newobject)
{
	static bool NeedReBuild = true;

	if (NeedReBuild || newobject)
	{
		NeedReBuild = false;
		return SIM_SOLVER_SUCCESS;
	}

	Lib1Func();

//	using namespace hpp::fcl;
//
//	// Define boxes
//	Box shape1(1, 1, 1);
//	Box shape2(1, 1, 1);
//
//	// Define transforms
//	Transform3f T1 = Transform3f::Identity();
//	Transform3f T2 = Transform3f::Identity();
//
//	// Compute collision
//	CollisionRequest req;
//	req.enable_cached_gjk_guess = true;
//	req.distance_upper_bound = 1e-6;
//	CollisionResult res;
//	ComputeCollision collide_functor(&shape1, &shape2);
//
//	T1.setTranslation(Vec3f(0, 0, 0));
//	res.clear();
//	std::cout << collide(&shape1, T1, &shape2, T2, req, res) << std::endl;
//	res.clear();
//	collide_functor(T1, T2, req, res);
//
//	T1.setTranslation(Vec3f(2, 0, 0));
//	res.clear();
//	std::cout << collide(&shape1, T1, &shape2, T2, req, res) << std::endl;
//	res.clear();
//	collide_functor(T1, T2, req, res);

	return SIM_SOLVER_SUCCESS;
}
