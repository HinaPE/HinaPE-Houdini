#include <UT/UT_DSOVersion.h> // Very Important!!! Include this!!!

#include <Collision/GAS_Hina_CollisionSolve.h>
#include <Collision/SIM_Hina_RigidBodyCollider.h>

#include <Common/SIM_Hina_Particles.h>
#include <Common/GAS_Hina_BuildNeighborLists.h>
#include <Common/GAS_Hina_ClearForce.h>
#include <Common/GAS_Hina_GravityForce.h>
#include <Common/GAS_Hina_SemiImplicitEuler.h>
#include <Common/GAS_Hina_UpdateDensity.h>
#include <Common/GAS_Hina_VolumeParticleEmitter.h>
#include <Common/GAS_Hina_SubStep.h>

#include <DFSPH/SIM_Hina_DFSPHParticles.h>
#include <DFSPH/GAS_Hina_DFSPHSolver.h>

#include <Visualizer/SIM_Hina_ParticlesVisualizer.h>

void initializeSIM(void *)
{
	// Collision
	IMPLEMENT_DATAFACTORY(GAS_Hina_CollisionSolve);
	IMPLEMENT_DATAFACTORY(SIM_Hina_RigidBodyCollider);

	// Common
	IMPLEMENT_DATAFACTORY(GAS_Hina_BuildNeighborLists)
	IMPLEMENT_DATAFACTORY(SIM_Hina_Particles)
	IMPLEMENT_DATAFACTORY(GAS_Hina_CommitCache)
	IMPLEMENT_DATAFACTORY(GAS_Hina_ClearForce)
	IMPLEMENT_DATAFACTORY(GAS_Hina_GravityForce)
	IMPLEMENT_DATAFACTORY(GAS_Hina_SemiImplicitEuler)
	IMPLEMENT_DATAFACTORY(GAS_Hina_UpdateDensity)
	IMPLEMENT_DATAFACTORY(GAS_Hina_VolumeParticleEmitter)
	IMPLEMENT_DATAFACTORY(GAS_Hina_SubStep)

	// DFSPH
	IMPLEMENT_DATAFACTORY(SIM_Hina_DFSPHParticles)
	IMPLEMENT_DATAFACTORY(GAS_Hina_DFSPHSolver)

	// Visualizer
//	IMPLEMENT_DATAFACTORY(SIM_Hina_ParticlesVisualizer)
}
