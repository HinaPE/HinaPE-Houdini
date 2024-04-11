#include "GAS_Hina_GridDiffusion.h"

GAS_HINA_SUBSOLVER_IMPLEMENT(
		GridDiffusion,
		true,
		false,
		ACTIVATE_GAS_SOURCE \
        ACTIVATE_GAS_DENSITY \
        ACTIVATE_GAS_TEMPERATURE \
        ACTIVATE_GAS_COLLISION \
        ACTIVATE_GAS_VELOCITY \
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

	if (!S || !D || !T || !C || !V)
		return false;

	if (!V->isFaceSampled())
		return false;

	SIM_RawField VX_Copy = *V->getXField();
	SIM_RawField VY_Copy = *V->getYField();
	SIM_RawField VZ_Copy = *V->getZField();
	_diffusion(timestep, 0.1, V->getXField(), &VX_Copy, nullptr, nullptr);

	return true;
}
void GAS_Hina_GridDiffusion::_diffusionPartial(float dt, float coefficient, SIM_RawField *OutField, const SIM_RawField *InField, const SIM_RawField *BoundarySDF, const SIM_RawField *FluidSDF, const UT_JobInfo &info)
{

}
