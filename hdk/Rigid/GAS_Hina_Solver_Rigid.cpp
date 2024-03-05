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
	this->_total_rb_count = 0;
	this->_solved_rb_count = 0;
}
void GAS_Hina_Solver_Rigid::_makeEqual(const GAS_Hina_Solver_Rigid *src)
{
	this->inited = src->inited;
	this->world = src->world;
	this->_total_rb_count = src->_total_rb_count;
	this->_solved_rb_count = src->_solved_rb_count;
}
bool GAS_Hina_Solver_Rigid::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_Hina_RigidBody *RB = SIM_DATA_GET(*obj, SIM_Hina_RigidBody::DATANAME, SIM_Hina_RigidBody);
	if (RB == nullptr)
		return true;

	if (!inited)
		init_data(RB, obj);

	if (check_need_solve())
	{
		world->update(timestep); // We Only Want to Update Once per Substep
		for (auto &rb: FetchAllRigidBodies(obj))
		{
			auto pos = rb->rb->getTransform().getPosition();
//			std::cout << "Rigid Body Position: " << pos.x << " " << pos.y << " " << pos.z << std::endl;
		}
	}

	return true;
}
void GAS_Hina_Solver_Rigid::init_data(SIM_Hina_RigidBody *RB, SIM_Object *obj)
{
	if (!world)
		world = physicsCommon.createPhysicsWorld();

	_total_rb_count = FetchAllRigidBodies(obj).size();
	InitAllRigidBodies(obj, physicsCommon, world);

	inited = true;
}
bool GAS_Hina_Solver_Rigid::check_need_solve()
{
	++_solved_rb_count;
	if (_solved_rb_count < _total_rb_count)
		return false;
	_solved_rb_count = 0;
	return true;
}
