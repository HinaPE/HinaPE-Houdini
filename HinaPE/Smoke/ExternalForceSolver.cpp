#include "ExternalForceSolver.h"

void HinaPE::ExternalForceSolver::_apply_gravityPartial(float dt, SIM_RawField *V_Y, const UT_JobInfo &info)
{
	UT_VoxelArrayIteratorF vit;
	V_Y->getPartialRange(vit, info);
	vit.setCompressOnExit(true);
	for (vit.rewind(); !vit.atEnd(); vit.advance())
	{
		auto value = V_Y->field()->getValue(vit.x(), vit.y(), vit.z());
		vit.setValue(value + dt * GRAVITY);
	}
}
void HinaPE::ExternalForceSolver::_apply_buoyancyPartial(float dt, SIM_RawField *V_Y, const SIM_RawField *D, const SIM_RawField *T, fpreal t_amb, const UT_JobInfo &info)
{
	UT_VoxelArrayIteratorF vit;
	V_Y->getPartialRange(vit, info);
	vit.setCompressOnExit(true);
	for (vit.rewind(); !vit.atEnd(); vit.advance())
	{
		auto pos = V_Y->indexToPos({vit.x(), vit.y(), vit.z()});
		auto fBuoy = DENSITY_FACTOR * D->field()->getValue(vit.x(), vit.y(), vit.z()) + TEMPERATURE_FACTOR * (T->field()->getValue(vit.x(), vit.y(), vit.z()) - t_amb);
		auto value = V_Y->field()->getValue(vit.x(), vit.y(), vit.z());
		vit.setValue(value + dt * fBuoy);
	}
}