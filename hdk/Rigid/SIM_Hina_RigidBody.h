#ifndef HINAPE_SIM_HINA_RIGIDBODY_H
#define HINAPE_SIM_HINA_RIGIDBODY_H

#include <SIM_Hina_Generator.h>

#include "reactphysics3d/reactphysics3d.h"

SIM_HINA_DATA_CLASS(
		RigidBody,

		reactphysics3d::RigidBody *rb;
		reactphysics3d::Transform WorldTransform;
)

#endif //HINAPE_SIM_HINA_RIGIDBODY_H
