#include "GAS_Hina_GridAdvect.h"

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
	_advect(timestep, D->getField(), &D_Copy, V->getXField(), V->getYField(), V->getZField());

	return true;
}
void GAS_Hina_GridAdvect::_advectPartial(float dt, SIM_RawField *OutField, const SIM_RawField *InField, const SIM_RawField *Flow_X, const SIM_RawField *Flow_Y, const SIM_RawField *Flow_Z, const UT_JobInfo &info)
{
	UT_VoxelArrayIteratorF vit;
	OutField->getPartialRange(vit, info);
	vit.setCompressOnExit(true);
	for (vit.rewind(); !vit.atEnd(); vit.advance())
	{
		UT_Vector3 pt0, pt1;
		OutField->indexToPos(vit.x(), vit.y(), vit.z(), pt0);
		pt1 = _back_trace(dt, pt0, Flow_X, Flow_Y, Flow_Z);
		fpreal32 value = InField->getValue(pt1);
		vit.setValue(value);
	}
}
UT_Vector3 GAS_Hina_GridAdvect::_back_trace(float dt, const UT_Vector3 &pt, const SIM_RawField *Flow_X, const SIM_RawField *Flow_Y, const SIM_RawField *Flow_Z)
{
	UT_BoundingBox bbox(Flow_X->getBBoxOrig(), Flow_X->getBBoxOrig() + Flow_X->getBBoxSize());

	float remaining_time = dt;
	UT_Vector3 pt0 = pt;
	UT_Vector3 pt1 = pt;
	const float h = std::min(Flow_X->getVoxelSize().x(), std::min(Flow_X->getVoxelSize().y(), Flow_X->getVoxelSize().z()));

	while (remaining_time > std::numeric_limits<float>::epsilon())
	{
		UT_Vector3 vel0 = UT_Vector3(Flow_X->getValue(pt0), Flow_Y->getValue(pt0), Flow_Z->getValue(pt0));
		const float num_substeps = std::max(1.0f, std::ceil(vel0.length() * remaining_time / h));
		dt = remaining_time / num_substeps;

		// mid-point rule
		UT_Vector3 mid_pos = pt0 - 0.5f * dt * vel0;
		UT_Vector3 mid_vel = UT_Vector3(Flow_X->getValue(mid_pos), Flow_Y->getValue(mid_pos), Flow_Z->getValue(mid_pos));
		pt1 = pt0 - dt * mid_vel;

		// Boundary handling
		if (pt1.x() < bbox.xmin())
			pt1.x() = bbox.xmin();
		if (pt1.x() > bbox.xmax())
			pt1.x() = bbox.xmax();
		if (pt1.y() < bbox.ymin())
			pt1.y() = bbox.ymin();
		if (pt1.y() > bbox.ymax())
			pt1.y() = bbox.ymax();
		if (pt1.z() < bbox.zmin())
			pt1.z() = bbox.zmin();
		if (pt1.z() > bbox.zmax())
			pt1.z() = bbox.zmax();

		remaining_time -= dt;
		pt0 = pt1;
	}

	return pt1;
}
