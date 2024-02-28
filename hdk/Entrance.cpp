#include <UT/UT_DSOVersion.h> // Very Important!!! Include this!!!

#include <Base/SIM_Hina_Particles.h>
#include <Base/GAS_Hina_VolumeParticleEmitter.h>
#include <Akinci2012/SIM_Hina_Akinci_Particles.h>
#include <DFSPH/SIM_Hina_DFSPH_Particles.h>
#include <DFSPH/GAS_Hina_DFSPH_Solver.h>
#include <DFSPH/GAS_Hina_DFSPH_SolverSPlisHSPlasH.h>

#include "GAS_Hina_SubStep.h"

void initializeSIM(void *)
{
	// Completed Classes
	IMPLEMENT_DATAFACTORY(GAS_Hina_SubStep)
	IMPLEMENT_DATAFACTORY(SIM_Hina_Particles)
	IMPLEMENT_DATAFACTORY(GAS_Hina_VolumeParticleEmitter)
	IMPLEMENT_DATAFACTORY(SIM_Hina_Akinci_Particles)
	IMPLEMENT_DATAFACTORY(SIM_Hina_DFSPH_Particles)
	IMPLEMENT_DATAFACTORY(GAS_Hina_DFSPH_Solver)
	IMPLEMENT_DATAFACTORY(GAS_Hina_DFSPH_SolverSPlisHSPlasH)

}
