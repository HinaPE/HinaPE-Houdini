#include "GAS_Hina_Solver_Rigid.h"

GAS_HINA_SUBSOLVER_IMPLEMENT(
		Solver_Rigid,
		true,
		false,
)
void GAS_Hina_Solver_Rigid::_init()
{
	this->inited = false;
	this->world = nullptr;
}
void GAS_Hina_Solver_Rigid::_makeEqual(const GAS_Hina_Solver_Rigid *src)
{
	this->inited = src->inited;
	this->world = src->world;
}
bool GAS_Hina_Solver_Rigid::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_Hina_RigidBody *RB = SIM_DATA_GET(*obj, SIM_Hina_RigidBody::DATANAME, SIM_Hina_RigidBody);
	if (RB == nullptr)
		return true;

	if (!inited)
		init_data(RB, obj);

	world->update(timestep);

	return true;
}
void GAS_Hina_Solver_Rigid::init_data(SIM_Hina_RigidBody *RB, SIM_Object *obj)
{
	if (!world)
		world = physicsCommon.createPhysicsWorld();

	RB->rb = world->createRigidBody(RB->WorldTransform);

	inited = true;
}
