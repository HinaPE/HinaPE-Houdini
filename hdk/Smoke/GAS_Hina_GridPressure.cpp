#include "GAS_Hina_GridPressure.h"

GAS_HINA_SUBSOLVER_IMPLEMENT(
		GridPressure,
		true,
		false,
		ACTIVATE_GAS_SOURCE \
        ACTIVATE_GAS_DENSITY \
        ACTIVATE_GAS_TEMPERATURE \
        ACTIVATE_GAS_COLLISION \
        ACTIVATE_GAS_VELOCITY \
        ACTIVATE_GAS_MARKER \
)

void GAS_Hina_GridPressure::_init() {}
void GAS_Hina_GridPressure::_makeEqual(const GAS_Hina_GridPressure *src) {}
bool GAS_Hina_GridPressure::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	return true;
}
