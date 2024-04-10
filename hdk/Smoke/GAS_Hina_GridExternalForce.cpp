#include "GAS_Hina_GridExternalForce.h"

GAS_HINA_SUBSOLVER_IMPLEMENT(
		GridExternalForce,
		true,
		false,
		HINA_FLOAT_PARAMETER(Gravity, 9.8) \
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

	_apply(timestep, V->getXField(), V->getYField(), V->getZField());

	return true;
}
void GAS_Hina_GridExternalForce::_applyPartial(float dt, SIM_RawField *V_X, SIM_RawField *V_Y, SIM_RawField *V_Z, const UT_JobInfo &info)
{
	// Apply Gravity
	UT_VoxelArrayIteratorF vit;
	V_Y->getPartialRange(vit, info);
	vit.setCompressOnExit(true);
	for (vit.rewind(); !vit.atEnd(); vit.advance())
	{
		auto value = V_Y->field()->getValue(vit.x(), vit.y(), vit.z());
		V_Y->fieldNC()->setValue(vit.x(), vit.y(), vit.z(), value + dt * getGravity());
	}
}
