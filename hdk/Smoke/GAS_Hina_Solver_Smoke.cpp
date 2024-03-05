#include "GAS_Hina_Solver_Smoke.h"

GAS_HINA_SUBSOLVER_IMPLEMENT(
		Solver_Smoke,
		true,
		false,
)

void GAS_Hina_Solver_Smoke::_init() {}
void GAS_Hina_Solver_Smoke::_makeEqual(const GAS_Hina_Solver_Smoke *src) {}
bool GAS_Hina_Solver_Smoke::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	return true;
}
