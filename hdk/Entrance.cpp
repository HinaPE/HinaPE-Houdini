#include <UT/UT_DSOVersion.h> // Very Important!!! Include this!!!
#include <GAS_Hina_SubStep.h>

#include "Particles/SIM_FluidParticle.h"
#include "Particles/GAS_ConfigureFluidParticleData.h"
#include "Emitter/GAS_VolumeParticleEmitter.h"
#include "Solvers/GAS_ParticleIntegrator.h"
#include "Neighbor/GAS_BuildNeighborLists.h"
#include "Density/GAS_ParticleDensity.h"
#include "Viscosity/GAS_ParticleViscosity.h"

#include "Boundary/SIM_SemiAnalyticalCollider.h"
#include "Boundary/GAS_ConfigureSemiAnalyticalCollider.h"
#include "Boundary/GAS_SetCollider.h"
#include "Boundary/SemiAnalytical/GAS_SetAABB.h"


void initializeSIM(void *)
{
	// Completed Classes
	//IMPLEMENT_DATAFACTORY(GAS_Hina_SubStep)

    // Incomplete Classes
    IMPLEMENT_DATAFACTORY(GAS_ConfigureFluidParticleData)
    IMPLEMENT_DATAFACTORY(SIM_FluidParticle)
    IMPLEMENT_DATAFACTORY(GAS_VolumeParticleEmitter)
    IMPLEMENT_DATAFACTORY(GAS_ParticleIntegrator)
    IMPLEMENT_DATAFACTORY(GAS_BuildNeighborLists)
    IMPLEMENT_DATAFACTORY(GAS_ParticleDensity)
    IMPLEMENT_DATAFACTORY(GAS_ParticleViscosity)

    IMPLEMENT_DATAFACTORY(SIM_SemiAnalyticalCollider)
    IMPLEMENT_DATAFACTORY(GAS_ConfigureSemiAnalyticalCollider)
    IMPLEMENT_DATAFACTORY(GAS_SetCollider)
    IMPLEMENT_DATAFACTORY(GAS_SetAABB)
}
