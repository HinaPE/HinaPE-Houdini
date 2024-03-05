#ifndef HINAPE_GAS_HINA_SOLVER_RIGID_H
#define HINAPE_GAS_HINA_SOLVER_RIGID_H

#include <SIM_Hina_Generator.h>

#include <Rigid/SIM_Hina_RigidBody.h>
#include "reactphysics3d/reactphysics3d.h"

GAS_HINA_SUBSOLVER_CLASS(
		Solver_Rigid,
		reactphysics3d::PhysicsCommon physicsCommon;
		reactphysics3d::PhysicsWorld *world;

		void init_data(SIM_Hina_RigidBody*, SIM_Object *);
		bool check_need_solve();
		bool inited;
		int _total_rb_count;
		int _solved_rb_count;
)

#endif //HINAPE_GAS_HINA_SOLVER_RIGID_H
