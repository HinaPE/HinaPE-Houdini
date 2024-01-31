#include "SIM_Hina_RigidBodyCollider.h"

SIM_HINA_COLLIDER_IMPLEMENT(
		RigidBodyCollider,
		HINA_FLOAT_PARAMETER(RestitutionCoefficient, .01) \
)

void SIM_Hina_RigidBodyCollider::_init()
{
	this->InnerPtr = nullptr;
}
void SIM_Hina_RigidBodyCollider::_makeEqual(const SIM_Hina_RigidBodyCollider *src)
{
	this->InnerPtr = src->InnerPtr;
}
