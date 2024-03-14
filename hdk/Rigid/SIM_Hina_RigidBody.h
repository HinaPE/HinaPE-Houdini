#ifndef HINAPE_SIM_HINA_RIGIDBODY_H
#define HINAPE_SIM_HINA_RIGIDBODY_H

#include <SIM_Hina_Generator.h>

#include "reactphysics3d/reactphysics3d.h"

SIM_HINA_DATA_CLASS(
		RigidBody,
		HINA_GETSET_PARAMETER(TargetGeometryDATANAME, GETSET_DATA_FUNCS_S) \
		HINA_GETSET_PARAMETER(IsDynamic, GETSET_DATA_FUNCS_B) \
		HINA_GETSET_PARAMETER(Bounciness, GETSET_DATA_FUNCS_F) \
		HINA_GETSET_PARAMETER(Friction, GETSET_DATA_FUNCS_F)
		reactphysics3d::RigidBody *rb;
		int b_set_index; // FOR FLUID SOLVER

		float *V; // ONLY USED FOR GUIDE GEOMETRY
		int *I; // ONLY USED FOR GUIDE GEOMETRY
		reactphysics3d::PolygonVertexArray::PolygonFace *F; // ONLY USED FOR GUIDE GEOMETRY
		int v_size; // ONLY USED FOR GUIDE GEOMETRY
		int i_size; // ONLY USED FOR GUIDE GEOMETRY
		int F_size; // ONLY USED FOR GUIDE GEOMETRY
		reactphysics3d::Transform WorldTransformCache; // ONLY USED FOR GUIDE GEOMETRY

		SIM_Guide *createGuideObjectSubclass() const override;
		void buildGuideGeometrySubclass(const SIM_RootData &root, const SIM_Options &options, const GU_DetailHandle &gdh, UT_DMatrix4 *xform, const SIM_Time &t) const override;
)

auto FetchAllRigidBodies(SIM_Object *obj) -> std::vector<SIM_Hina_RigidBody *>;
void InitAllRigidBodies(SIM_Object *obj, reactphysics3d::PhysicsCommon &physicsCommon, reactphysics3d::PhysicsWorld *world);
void UpdateAllRigidBodies(SIM_Object *obj);

#endif //HINAPE_SIM_HINA_RIGIDBODY_H
