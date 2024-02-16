#include "GAS_Hina_UpdateVolume.h"
#include <Particles/SIM_Hina_Particles.h>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		UpdateVolume,
		true,
		false,
)
void GAS_Hina_UpdateVolume::_init() {}
void GAS_Hina_UpdateVolume::_makeEqual(const GAS_Hina_UpdateVolume *src) {}
bool GAS_Hina_UpdateVolume::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	// TODO: fluid have to be incompressible
	return true;
}
