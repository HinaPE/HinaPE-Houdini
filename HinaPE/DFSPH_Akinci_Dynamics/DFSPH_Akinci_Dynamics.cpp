#include "DFSPH_Akinci_Dynamics.h"

#include <numeric>
#include <utility>
#include <execution>

HinaPE::DFSPH_Akinci_DynamicsSolver::DFSPH_Akinci_DynamicsSolver(HinaPE::real _r, HinaPE::Vector _b)
		: DFSPH_AkinciSolver(_r, _b), world(physicsCommon.createPhysicsWorld()) {}

void HinaPE::DFSPH_Akinci_DynamicsSolver::Solve(HinaPE::real dt)
{
	resize();
	_update_dynamic_akinci_boundaries();

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

void HinaPE::DFSPH_Akinci_DynamicsSolver::_update_dynamic_akinci_boundaries()
{
	for (auto &Boundary: Boundaries)
	{
		std::transform(Boundary->x_init.begin(), Boundary->x_init.end(), Boundary->x.begin(), [&](Vector x) { return rowVecMult(x, Boundary->xform); });
		std::fill(Boundary->v.begin(), Boundary->v.end(), Vector{0, 0, 0});
		std::fill(Boundary->a.begin(), Boundary->a.end(), Vector{0, 0, 0});
	}

	if (!DynamicBoundariesInited)
	{
		for (auto &Boundary: Boundaries)
		{
			world->createRigidBody();
		}
	}
}
