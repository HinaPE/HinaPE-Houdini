#ifndef HINAPE_SIM_HINA_RIGIDBODYCOLLIDER_H
#define HINAPE_SIM_HINA_RIGIDBODYCOLLIDER_H

#include <SIM_Hina_Generator.h>

namespace CubbyFlow
{
template<size_t N>
class RigidBodyCollider;
}

SIM_HINA_COLLIDER_CLASS(
		RigidBodyCollider,
		std::shared_ptr<CubbyFlow::RigidBodyCollider<3>> InnerPtr;
		HINA_GETSET_PARAMETER(RestitutionCoefficient, GETSET_DATA_FUNCS_F)
)

#endif //HINAPE_SIM_HINA_RIGIDBODYCOLLIDER_H
