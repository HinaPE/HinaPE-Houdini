#include "GAS_Hina_Solver_Smoke.h"

#include <GAS/GAS_Advect.h>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		Solver_Smoke,
		true,
		false,

		ACTIVATE_GAS_VELOCITY \
        ACTIVATE_GAS_DENSITY \
        ACTIVATE_GAS_TEMPERATURE \
        ACTIVATE_GAS_COLLISION \
)

void GAS_Hina_Solver_Smoke::_init() {}
void GAS_Hina_Solver_Smoke::_makeEqual(const GAS_Hina_Solver_Smoke *src) {}
bool GAS_Hina_Solver_Smoke::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time dt)
{
	SIM_ScalarField *D = getScalarField(obj, GAS_NAME_DENSITY);
	SIM_ScalarField *T = getScalarField(obj, GAS_NAME_TEMPERATURE);
	SIM_ScalarField *C = getScalarField(obj, GAS_NAME_COLLISION);
	SIM_VectorField *V = getVectorField(obj, GAS_NAME_VELOCITY);

	return true;
}
