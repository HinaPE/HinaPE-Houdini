#include "GAS_Hina_GridExternalForce.h"

#include <HinaPE/Smoke/ExternalForceSolver.h>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		GridExternalForce,
		true,
		false,
		HINA_FLOAT_PARAMETER(Gravity, 9.8) \
        HINA_FLOAT_PARAMETER(DensityFactor, -0.000625) \
        HINA_FLOAT_PARAMETER(TemperatureFactor, 5.0) \
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
	SIM_ScalarField *S = getScalarField(obj, GAS_NAME_SOURCE);
	SIM_ScalarField *D = getScalarField(obj, GAS_NAME_DENSITY);
	SIM_ScalarField *T = getScalarField(obj, GAS_NAME_TEMPERATURE);
	SIM_ScalarField *C = getScalarField(obj, GAS_NAME_COLLISION);
	SIM_VectorField *V = getVectorField(obj, GAS_NAME_VELOCITY);

	if (!S || !D || !T || !C || !V)
		return false;

	if (!V->isFaceSampled())
		return false;

	static HinaPE::ExternalForceSolver _;
	_.GRAVITY = getGravity();
	_.DENSITY_FACTOR = getDensityFactor();
	_.TEMPERATURE_FACTOR = getTemperatureFactor();
	_._apply_gravity(timestep, V->getYField());
	fpreal t_amb = T->getField()->average() / T->getField()->getVoxelVolume();
	_._apply_buoyancy(timestep, V->getYField(), D->getField(), T->getField(), t_amb);

	return true;
}
