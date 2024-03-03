#include <UT/UT_DSOVersion.h> // Very Important!!! Include this!!!

#include <GAS_CFL_SubStep.h>
#include <Base/SIM_Hina_Particles.h>
#include <DFSPH/SIM_Hina_Particles_DFSPH.h>
#include <DFSPH/GAS_Hina_Solver_DFSPH.h>
#include <Boundary/SIM_Hina_Particles_Akinci.h>

void initializeSIM(void *)
{
	// Completed Classes
	IMPLEMENT_DATAFACTORY(GAS_CFL_SubStep)
	IMPLEMENT_DATAFACTORY(SIM_Hina_Particles)
	IMPLEMENT_DATAFACTORY(SIM_Hina_Particles_DFSPH)
	IMPLEMENT_DATAFACTORY(GAS_Hina_Solver_DFSPH)
	IMPLEMENT_DATAFACTORY(SIM_Hina_Particles_Akinci)
}
