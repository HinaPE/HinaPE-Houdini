#include <UT/UT_DSOVersion.h> // Very Important!!! Include this!!!

#include <Density/GAS_Hina_UpdateDensity.h>
#include <Density/GAS_Hina_UpdateDensityAkinci.h>

#include <Emitter/GAS_Hina_VolumeParticleEmitter.h>

#include <Neighbor/GAS_Hina_BuildNeighborLists.h>

#include <Particles/SIM_Hina_Akinci2012BoundaryParticles.h>
#include <Particles/SIM_Hina_DFSPHParticles.h>
#include <Particles/SIM_Hina_Particles.h>
#include <Particles/Visualizer/SIM_Hina_ParticlesVisualizer.h>

#include <Solvers/GAS_Hina_DFSPHSolver.h>
#include <GAS_Hina_SubStep.h>


void initializeSIM(void *)
{
	// Completed Classes
	IMPLEMENT_DATAFACTORY(GAS_Hina_SubStep)

	IMPLEMENT_DATAFACTORY(GAS_Hina_UpdateDensity)
	IMPLEMENT_DATAFACTORY(GAS_Hina_UpdateDensityAkinci)

	IMPLEMENT_DATAFACTORY(GAS_Hina_VolumeParticleEmitter)

	IMPLEMENT_DATAFACTORY(GAS_Hina_BuildNeighborLists)

	IMPLEMENT_DATAFACTORY(SIM_Hina_Akinci2012BoundaryParticles)
	IMPLEMENT_DATAFACTORY(SIM_Hina_DFSPHParticles)
	IMPLEMENT_DATAFACTORY(SIM_Hina_Particles)
	IMPLEMENT_DATAFACTORY(SIM_Hina_ParticlesNeighborsVisualizer)

	// On Development
	IMPLEMENT_DATAFACTORY(GAS_Hina_DFSPHSolver)
}
