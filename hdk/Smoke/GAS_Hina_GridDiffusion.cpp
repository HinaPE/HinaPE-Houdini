#include "GAS_Hina_GridDiffusion.h"

#include <HinaPE/Smoke/DiffusionSolver.h>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		GridDiffusion,
		true,
		false,
		HINA_INT_PARAMETER(Diffusion, 0.1f) \
		ACTIVATE_GAS_SOURCE \
        ACTIVATE_GAS_DENSITY \
        ACTIVATE_GAS_TEMPERATURE \
        ACTIVATE_GAS_COLLISION \
        ACTIVATE_GAS_VELOCITY \
        ACTIVATE_GAS_MARKER \
)

void GAS_Hina_GridDiffusion::_init() {}
void GAS_Hina_GridDiffusion::_makeEqual(const GAS_Hina_GridDiffusion *src) {}
bool GAS_Hina_GridDiffusion::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_ScalarField *S = getScalarField(obj, GAS_NAME_SOURCE);
	SIM_ScalarField *D = getScalarField(obj, GAS_NAME_DENSITY);
	SIM_ScalarField *T = getScalarField(obj, GAS_NAME_TEMPERATURE);
	SIM_ScalarField *C = getScalarField(obj, GAS_NAME_COLLISION);
	SIM_VectorField *V = getVectorField(obj, GAS_NAME_VELOCITY);
	SIM_VectorField *M = getVectorField(obj, GAS_NAME_STENCIL);

	if (!S || !D || !T || !C || !V || !M)
		return false;

	if (!V->isFaceSampled())
		return false;

	static HinaPE::DiffusionSolver _;
	_.DIFFUSION = getDiffusion();
	_.solve(D->getField(), C->getField());

	return true;
}
