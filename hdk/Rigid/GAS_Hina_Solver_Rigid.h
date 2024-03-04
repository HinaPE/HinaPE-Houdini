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
		bool inited;
)

#endif //HINAPE_GAS_HINA_SOLVER_RIGID_H
