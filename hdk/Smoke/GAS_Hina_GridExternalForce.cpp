#include "GAS_Hina_GridExternalForce.h"

GAS_HINA_SUBSOLVER_IMPLEMENT(
		GridExternalForce,
		true,
		false,
		ACTIVATE_GAS_SOURCE \
        ACTIVATE_GAS_DENSITY \
        ACTIVATE_GAS_TEMPERATURE \
        ACTIVATE_GAS_COLLISION \
        ACTIVATE_GAS_VELOCITY \
)

void GAS_Hina_GridExternalForce::_init() {}
void GAS_Hina_GridExternalForce::_makeEqual(const GAS_Hina_GridExternalForce *src) {}
bool GAS_Hina_GridExternalForce::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	return true;
}
