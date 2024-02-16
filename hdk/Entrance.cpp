#include <UT/UT_DSOVersion.h> // Very Important!!! Include this!!!

#include <Advect/GAS_Hina_AdvectPos.h>
#include <Advect/GAS_HIna_AdvectVel.h>
#include <Advect/GAS_Hina_SemiImplicitEuler.h>
#include <Advect/GAS_Hina_SubStep.h>

#include <Collision/GAS_Hina_CollisionSolve.h>
#include <Collision/GAS_Hina_EnforceBoundary.h>
#include <Collision/SIM_Hina_RigidBodyCollider.h>

#include <Density/GAS_Hina_UpdateDensity.h>

#include <Emitter/GAS_Hina_VolumeParticleEmitter.h>

#include <Force/NonPressure/GAS_Hina_ClearForce.h>
#include <Force/NonPressure/GAS_Hina_GravityForce.h>

#include <Neighbor/GAS_Hina_BuildNeighborLists.h>

#include <Particles/SIM_Hina_Akinci2012BoundaryParticles.h>
#include <Particles/SIM_Hina_DFSPHParticles.h>
#include <Particles/SIM_Hina_Particles.h>
#include <Particles/Visualizer/SIM_Hina_ParticlesVisualizer.h>

#include <_Temp/GAS_Hina_DFSPHSolver.h>
#include <_Temp/GAS_TEST.h>


void initializeSIM(void *)
{
	IMPLEMENT_DATAFACTORY(GAS_Hina_AdvectPos)
	IMPLEMENT_DATAFACTORY(GAS_Hina_AdvectVel)
	IMPLEMENT_DATAFACTORY(GAS_Hina_SemiImplicitEuler)
	IMPLEMENT_DATAFACTORY(GAS_Hina_SubStep)

	IMPLEMENT_DATAFACTORY(GAS_Hina_CollisionSolve);
	IMPLEMENT_DATAFACTORY(GAS_Hina_EnforceBoundary);
	IMPLEMENT_DATAFACTORY(SIM_Hina_RigidBodyCollider);

	IMPLEMENT_DATAFACTORY(GAS_Hina_UpdateDensity)

	IMPLEMENT_DATAFACTORY(GAS_Hina_VolumeParticleEmitter)

	IMPLEMENT_DATAFACTORY(GAS_Hina_ClearForce)
	IMPLEMENT_DATAFACTORY(GAS_Hina_GravityForce)

	IMPLEMENT_DATAFACTORY(GAS_Hina_BuildNeighborLists)

	IMPLEMENT_DATAFACTORY(SIM_Hina_Akinci2012BoundaryParticles)
	IMPLEMENT_DATAFACTORY(SIM_Hina_DFSPHParticles)
	IMPLEMENT_DATAFACTORY(SIM_Hina_Particles)
	IMPLEMENT_DATAFACTORY(GAS_Hina_CommitCache)
	IMPLEMENT_DATAFACTORY(SIM_Hina_ParticlesNeighborsVisualizer)


	IMPLEMENT_DATAFACTORY(GAS_Hina_DFSPHSolver)
	IMPLEMENT_DATAFACTORY(GAS_Hina_TEST)
}
