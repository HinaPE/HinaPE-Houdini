#include "GAS_Hina_GridBoundarySolver.h"

#include <HinaPE/Smoke/BoundarySolver.h>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		GridBoundarySolver,
		true,
		false,
		HINA_INT_PARAMETER(Iter, 4) \
        ACTIVATE_GAS_SOURCE \
        ACTIVATE_GAS_DENSITY \
        ACTIVATE_GAS_TEMPERATURE \
        ACTIVATE_GAS_COLLISION \
        ACTIVATE_GAS_VELOCITY \
)

void GAS_Hina_GridBoundarySolver::_init() {}
void GAS_Hina_GridBoundarySolver::_makeEqual(const GAS_Hina_GridBoundarySolver *src) {}
bool GAS_Hina_GridBoundarySolver::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
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

	static HinaPE::BoundarySolver _;
	_.solve(V->getXField(), V->getYField(), V->getZField(), C->getField(), getIter());

	return true;
}
