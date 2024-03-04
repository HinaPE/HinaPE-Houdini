#include "DFSPH_Akinci_Dynamics.h"

#include <numeric>
#include <utility>
#include <execution>

HinaPE::DFSPH_Akinci_DynamicsSolver::DFSPH_Akinci_DynamicsSolver(HinaPE::real _r, HinaPE::Vector _b)
		: DFSPH_AkinciSolver(_r, _b), world(physicsCommon.createPhysicsWorld()) {}

void HinaPE::DFSPH_Akinci_DynamicsSolver::Solve(HinaPE::real dt)
{
	resize();
	update_akinci_boundaries();

	build_neighbors();
	compute_density();
	compute_factor();
	divergence_solve(dt);
	non_pressure_force();
	predict_velocity(dt);
	pressure_solve(dt);
	advect(dt);
	enforce_boundary();
}
void HinaPE::DFSPH_Akinci_DynamicsSolver::update_akinci_boundaries()
{
	if (!DynamicBoundariesInited)
	{
		for (const auto &Boundary: Boundaries)
		{
			reactphysics3d::Vector3 p{Boundary->pos.x(), Boundary->pos.y(), Boundary->pos.z()};
			reactphysics3d::Quaternion q{Boundary->quat.x(), Boundary->quat.y(), Boundary->quat.z(), Boundary->quat.w()};
			reactphysics3d::Transform t(p, q);
			auto *rb = world->createRigidBody(t);
			rb->setMass(5);
			rb->setType(reactphysics3d::BodyType::DYNAMIC);

//			physicsCommon.createConvexMeshShape();
//			rb->addCollider(reactphysics3d::SphereShape(Boundary->radius), reactphysics3d::Transform::identity());
//			rb->applyWorldForceAtCenterOfMass();
		}
	}

	for (auto &Boundary: Boundaries)
	{
		std::transform(Boundary->x_init.begin(), Boundary->x_init.end(), Boundary->x.begin(), [&](Vector x) { return rowVecMult(x, Boundary->xform); });
		std::fill(Boundary->v.begin(), Boundary->v.end(), Vector{0, 0, 0});
		std::fill(Boundary->a.begin(), Boundary->a.end(), Vector{0, 0, 0});
	}
}
