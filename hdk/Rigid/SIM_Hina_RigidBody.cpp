#include "SIM_Hina_RigidBody.h"

SIM_HINA_DATA_IMPLEMENT(
		RigidBody,
		true
)

void SIM_Hina_RigidBody::_init_RigidBody()
{
	this->rb = nullptr;
	this->WorldTransform.setToIdentity();
}
void SIM_Hina_RigidBody::_makeEqual_RigidBody(const SIM_Hina_RigidBody *src)
{
	this->rb = src->rb;
	this->WorldTransform = src->WorldTransform;
}
