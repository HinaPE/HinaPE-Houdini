#include "Smoke.h"
#include <UT/UT_MatrixSolver.h>
#include <UT/UT_ParallelUtil.h>
#include <UT/UT_ThreadedAlgorithm.h>
#include <UT/UT_Interrupt.h>
#include <memory>

void HinaPE::SmokeNativeSolver::Init(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V) {}
void HinaPE::SmokeNativeSolver::Solve(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V)
{
	emit(dt, D, T, V);
	non_pressure(dt, D, T, V);
	pressure(dt, D, T, V);
	advect(dt, D, T, V);
}

void HinaPE::SmokeNativeSolver::emit(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V)
{
	_emit_density(D);
	_emit_temperature(T);
}
void HinaPE::SmokeNativeSolver::non_pressure(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V)
{
	_apply_buoyancy_force(dt, D, T, V);
}
void HinaPE::SmokeNativeSolver::pressure(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V)
{

}
void HinaPE::SmokeNativeSolver::advect(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V)
{
	D_copy = SIM_RawField(*D->getField());
	T_copy = SIM_RawField(*T->getField());
	Vx_copy = SIM_RawField(*V->getXField());
	Vy_copy = SIM_RawField(*V->getYField());
	Vz_copy = SIM_RawField(*V->getZField());
	_advect_field(dt, D->getField(), &D_copy, V);
	_advect_field(dt, T->getField(), &T_copy, V);
	_advect_field(dt, V->getXField(), &Vx_copy, V);
	_advect_field(dt, V->getYField(), &Vy_copy, V);
	_advect_field(dt, V->getZField(), &Vz_copy, V);
}
void HinaPE::SmokeNativeSolver::_emit_densityPartial(SIM_ScalarField *D, const UT_JobInfo &info)
{
	auto center = SIMfieldUtilsGetCenter(D);

	UT_VoxelArrayIteratorF vit;
	D->getField()->getPartialRange(vit, info);
	vit.setCompressOnExit(true);
	for (vit.rewind(); !vit.atEnd(); vit.advance())
	{
		UT_Vector3 pos;
		D->indexToPos(vit.x(), vit.y(), vit.z(), pos);
		if ((pos - center).length() < 0.5f)
			vit.setValue(1.f);
	}
}
void HinaPE::SmokeNativeSolver::_emit_temperaturePartial(SIM_ScalarField *T, const UT_JobInfo &info)
{
	auto center = SIMfieldUtilsGetCenter(T);

	UT_VoxelArrayIteratorF vit;
	T->getField()->getPartialRange(vit, info);
	vit.setCompressOnExit(true);
	for (vit.rewind(); !vit.atEnd(); vit.advance())
	{
		UT_Vector3 pos;
		T->indexToPos(vit.x(), vit.y(), vit.z(), pos);
		if ((pos - center).length() < 0.5f)
			vit.setValue(1.f);
	}
}
void HinaPE::SmokeNativeSolver::_apply_buoyancy_forcePartial(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, const UT_JobInfo &info)
{
	int axis = 1;
	UT_VoxelArrayIteratorF vit;
	V->getField(axis)->getPartialRange(vit, info);
	vit.setCompressOnExit(true);

	for (vit.rewind(); !vit.atEnd(); vit.advance())
	{
		UT_Vector3 vel_pos;
		V->getField(axis)->indexToPos(vit.x(), vit.y(), vit.z(), vel_pos);

		fpreal32 den = D->getValue(vel_pos);
		fpreal32 tem = T->getValue(vel_pos);

		const fpreal32 f_buoyancy = BUOYANCY_SMOKE_DENSITY_FACTOR * den + BUOYANCY_SMOKE_TEMPERATURE_FACTOR * tem;
//		vit.setValue(vit.getValue() + dt * f_buoyancy);

		if (tem > 0)
			vit.setValue(1);
	}
}
void HinaPE::SmokeNativeSolver::_advect_fieldPartial(float dt, SIM_RawField *output, const SIM_RawField *input, SIM_VectorField *flow, const UT_JobInfo &info)
{
	UT_VoxelArrayIteratorF vit;
	output->getPartialRange(vit, info);
	vit.setCompressOnExit(true);
	for (vit.rewind(); !vit.atEnd(); vit.advance())
	{
		UT_Vector3 pt0, pt1;
		output->indexToPos(vit.x(), vit.y(), vit.z(), pt0);
		pt1 = _back_trace(dt, pt0, flow);
		fpreal32 value = input->getValue(pt1);
		vit.setValue(value);
	}
}
auto HinaPE::SmokeNativeSolver::_back_trace(float dt, const UT_Vector3 &pt, const SIM_VectorField *V) -> UT_Vector3
{
	float remaining_time = dt;
	UT_Vector3 pt0 = pt;
	UT_Vector3 pt1 = pt;
	const float h = std::min(V->getVoxelSize().x(), std::min(V->getVoxelSize().y(), V->getVoxelSize().z()));
	UT_BoundingBox bbox;
	V->getBBox(bbox);

	while (remaining_time > std::numeric_limits<float>::epsilon())
	{
		UT_Vector3 vel0 = V->getValue(pt0);
		const float num_substeps = std::max(1.0f, std::ceil(vel0.length() * remaining_time / h));
		dt = remaining_time / num_substeps;

		// mid-point rule
		UT_Vector3 mid_pos = pt0 - 0.5f * dt * vel0;
		UT_Vector3 mid_vel = V->getValue(mid_pos);
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
