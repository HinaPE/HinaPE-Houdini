#include "GAS_Hina_GridAdvect.h"
#include <HinaPE/Smoke/AdvectionSolver.h>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		GridAdvect,
		true,
		false,
		ACTIVATE_GAS_SOURCE \
        ACTIVATE_GAS_DENSITY \
        ACTIVATE_GAS_TEMPERATURE \
        ACTIVATE_GAS_COLLISION \
        ACTIVATE_GAS_VELOCITY \
)

void GAS_Hina_GridAdvect::_init() {}
void GAS_Hina_GridAdvect::_makeEqual(const GAS_Hina_GridAdvect *src) {}
bool GAS_Hina_GridAdvect::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_ScalarField *S = getScalarField(obj, GAS_NAME_SOURCE);
	SIM_ScalarField *D = getScalarField(obj, GAS_NAME_DENSITY);
	SIM_ScalarField *T = getScalarField(obj, GAS_NAME_TEMPERATURE);
	SIM_ScalarField *C = getScalarField(obj, GAS_NAME_COLLISION);
	SIM_VectorField *V = getVectorField(obj, GAS_NAME_VELOCITY);

	if (!S || !D || !T || !C || !V)
		return false;

	if (!V->isFaceSampled())
		return false;

	SIM_RawField D_Copy = *D->getField(); // would there be error?
	static HinaPE::AdvectionSolver _;
	_._advect(timestep, D->getField(), &D_Copy, V->getXField(), V->getYField(), V->getZField());

	return true;
}
