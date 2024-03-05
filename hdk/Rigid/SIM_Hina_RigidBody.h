#ifndef HINAPE_SIM_HINA_RIGIDBODY_H
#define HINAPE_SIM_HINA_RIGIDBODY_H

#include <SIM_Hina_Generator.h>

#include "reactphysics3d/reactphysics3d.h"

SIM_HINA_DATA_CLASS(
		RigidBody,
		HINA_GETSET_PARAMETER(IsDynamic, GETSET_DATA_FUNCS_B) \
		reactphysics3d::RigidBody *rb;
		float *V;
		int *I;
		reactphysics3d::PolygonVertexArray::PolygonFace *F;
		int v_size;
		int i_size;
		int F_size;
		reactphysics3d::Transform WorldTransformCache;

		SIM_Guide *createGuideObjectSubclass() const override;
		void buildGuideGeometrySubclass(const SIM_RootData &root, const SIM_Options &options, const GU_DetailHandle &gdh, UT_DMatrix4 *xform, const SIM_Time &t) const override;
)

auto FetchAllRigidBodies(SIM_Object *obj) -> std::vector<SIM_Hina_RigidBody *>;
void InitAllRigidBodies(SIM_Object *obj, reactphysics3d::PhysicsCommon &physicsCommon, reactphysics3d::PhysicsWorld *world);
void UpdateAllRigidBodies(SIM_Object *obj);

#endif //HINAPE_SIM_HINA_RIGIDBODY_H
