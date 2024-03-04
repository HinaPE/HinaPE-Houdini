#ifndef HINAPE_DFSPH_AKINCI_DYNAMICS_H
#define HINAPE_DFSPH_AKINCI_DYNAMICS_H

/**
 * DFSPH implementation, Akinci Boundary
 * https://github.com/erizmr/SPH_Taichi
 */

#include "DFSPH_Akinci/DFSPH_Akinci.h"
#include "reactphysics3d/reactphysics3d.h"
namespace HinaPE
{
struct DFSPH_Akinci_DynamicsSolver : public DFSPH_AkinciSolver
{
	DFSPH_Akinci_DynamicsSolver(real, Vector);
	void Solve(real dt) override;

private:
	void _update_dynamic_akinci_boundaries();
	reactphysics3d::PhysicsCommon physicsCommon;
	reactphysics3d::PhysicsWorld *world;
	bool DynamicBoundariesInited = false;
};
}

#endif //HINAPE_DFSPH_AKINCI_DYNAMICS_H
