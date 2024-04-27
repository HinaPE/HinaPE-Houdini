#include "GAS_Hina_Solver_Rigid.h"

#define ACTIVATE_GAS_GEOMETRY static PRM_Name GeometryName(GAS_NAME_GEOMETRY, SIM_GEOMETRY_DATANAME); static PRM_Default GeometryNameDefault(0, "GeometryOutput"); PRMS.emplace_back(PRM_STRING, 1, &GeometryName, &GeometryNameDefault);

GAS_HINA_SUBSOLVER_IMPLEMENT(
		Solver_Rigid,
		true,
		false,
		ACTIVATE_GAS_GEOMETRY
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
		UpdateAllRigidBodies(obj);

		SIM_ObjectArray affectors;
		obj->getAffectors(affectors, "SIM_RelationshipCollide");
		exint num_affectors = affectors.entries();
		for (int i = 0; i < num_affectors; ++i)
		{
			SIM_Object *obj_collider = affectors(i);
			SIM_Hina_RigidBody *rigidbody = SIM_DATA_GET(*obj_collider, SIM_Hina_RigidBody::DATANAME, SIM_Hina_RigidBody);
			if (rigidbody)
			{
				SIM_GeometryCopy *G = getGeometryCopy(obj_collider, GAS_NAME_GEOMETRY);
				if (!G) continue;

				UT_DMatrix4& xform = G->lockTransform();
				auto pos = rigidbody->rb->getTransform().getPosition();
				auto cm = rigidbody->center_of_mass;
				xform.setTranslates(UT_Vector3(pos.x - cm.x(), pos.y - cm.y(), pos.z - cm.z()));
				auto matrix = rigidbody->rb->getTransform().getOrientation().getMatrix().getTranspose();

				xform[0][0] = matrix[0][0];
				xform[0][1] = matrix[0][1];
				xform[0][2] = matrix[0][2];

				xform[1][0] = matrix[1][0];
				xform[1][1] = matrix[1][1];
				xform[1][2] = matrix[1][2];

				xform[2][0] = matrix[2][0];
				xform[2][1] = matrix[2][1];
				xform[2][2] = matrix[2][2];

				G->releaseTransform();
			}
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
