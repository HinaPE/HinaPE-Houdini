#include "GAS_TEST.h"

GAS_HINA_SUBSOLVER_IMPLEMENT(
		TEST,
		true,
		false,
)

void GAS_Hina_TEST::_init() {}
void GAS_Hina_TEST::_makeEqual(const GAS_Hina_TEST *src) {}
bool GAS_Hina_TEST::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	return true;
}
