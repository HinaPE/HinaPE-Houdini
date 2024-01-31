#ifndef HINAPE_SIM_HINA_RIGIDBODYCOLLIDER_H
#define HINAPE_SIM_HINA_RIGIDBODYCOLLIDER_H

#include <Common/SIM_Hina_Generator.h>
#include "CubbyFlow/Core/Geometry/RigidBodyCollider.hpp"

SIM_HINA_COLLIDER_CLASS(
		RigidBodyCollider,
		CubbyFlow::RigidBodyCollider3Ptr InnerPtr;
)

#endif //HINAPE_SIM_HINA_RIGIDBODYCOLLIDER_H
