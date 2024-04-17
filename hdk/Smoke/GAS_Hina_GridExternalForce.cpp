#include "GAS_Hina_GridExternalForce.h"

#include <HinaPE/Smoke/FieldUtils.h>

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

	_apply_gravity(timestep, V->getYField());
	fpreal t_amb = T->getField()->average() / T->getField()->getVoxelVolume();
	_apply_buoyancy(timestep, V->getYField(), D->getField(), T->getField(), t_amb);

	return true;
}
void GAS_Hina_GridExternalForce::_apply_gravityPartial(float dt, SIM_RawField *V_Y, const UT_JobInfo &info)
{
	UT_VoxelArrayIteratorF vit;
	V_Y->getPartialRange(vit, info);
	vit.setCompressOnExit(true);
	for (vit.rewind(); !vit.atEnd(); vit.advance())
	{
		auto value = V_Y->field()->getValue(vit.x(), vit.y(), vit.z());
		vit.setValue(value + dt * getGravity());
	}
}
void GAS_Hina_GridExternalForce::_apply_buoyancyPartial(float dt, SIM_RawField *V_Y, const SIM_RawField *D, const SIM_RawField *T, fpreal t_amb, const UT_JobInfo &info)
{
	UT_VoxelArrayIteratorF vit;
	V_Y->getPartialRange(vit, info);
	vit.setCompressOnExit(true);
	for (vit.rewind(); !vit.atEnd(); vit.advance())
	{
		auto pos = V_Y->indexToPos({vit.x(), vit.y(), vit.z()});
		auto fBuoy = getDensityFactor() * D->field()->getValue(vit.x(), vit.y(), vit.z()) + getTemperatureFactor() * (T->field()->getValue(vit.x(), vit.y(), vit.z()) - t_amb);
		auto value = V_Y->field()->getValue(vit.x(), vit.y(), vit.z());
		vit.setValue(value + dt * fBuoy);
	}
}
