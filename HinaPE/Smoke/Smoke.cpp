#include "Smoke.h"
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
//	advect(dt, D, T, V);
	advect_HDK(-dt, D, T, V);
	enforce_boundary(V);
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
	_apply_pressure_force(dt, V);
}
void HinaPE::SmokeNativeSolver::advect(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V)
{
	SIM_RawField D_target = SIM_RawField(*D->getField());
	SIM_RawField T_target = SIM_RawField(*T->getField());
	SIM_RawField Vx_target = SIM_RawField(*V->getXField());
	SIM_RawField Vy_target = SIM_RawField(*V->getYField());
	SIM_RawField Vz_target = SIM_RawField(*V->getZField());

	_advect_field(dt, &D_target, D->getField(), V);
	_advect_field(dt, &T_target, T->getField(), V);
	_advect_field(dt, &Vx_target, V->getXField(), V);
	_advect_field(dt, &Vy_target, V->getYField(), V);
	_advect_field(dt, &Vz_target, V->getZField(), V);

	(*D->getField()) = D_target;
	(*T->getField()) = T_target;
	(*V->getXField()) = Vx_target;
	(*V->getYField()) = Vy_target;
	(*V->getZField()) = Vz_target;
}
void HinaPE::SmokeNativeSolver::advect_HDK(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V)
{
	D->advect(V, dt, nullptr, SIM_FieldAdvection::SIM_ADVECT_RK4, 1.f);
	T->advect(V, dt, nullptr, SIM_FieldAdvection::SIM_ADVECT_RK4, 1.f);
	V->advect(V, dt, nullptr, SIM_FieldAdvection::SIM_ADVECT_RK4, 1.f);
}
void HinaPE::SmokeNativeSolver::enforce_boundary(SIM_VectorField *V)
{
	_enforce_boundary(V->getXField(), 0);
	_enforce_boundary(V->getYField(), 1);
	_enforce_boundary(V->getZField(), 2);
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
void HinaPE::SmokeNativeSolver::_apply_buoyancy_forcePartial(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *flow, const UT_JobInfo &info)
{
	int axis = 1;
	UT_VoxelArrayIteratorF vit;
	flow->getField(axis)->getPartialRange(vit, info);
	vit.setCompressOnExit(true);

	for (vit.rewind(); !vit.atEnd(); vit.advance())
	{
		UT_Vector3 vel_pos;
		flow->getField(axis)->indexToPos(vit.x(), vit.y(), vit.z(), vel_pos);

		fpreal32 den = D->getValue(vel_pos);
		fpreal32 tem = T->getValue(vel_pos);

		const fpreal32 f_buoyancy = BUOYANCY_SMOKE_DENSITY_FACTOR * den + BUOYANCY_SMOKE_TEMPERATURE_FACTOR * tem;
		vit.setValue(vit.getValue() + dt * f_buoyancy);
	}
}
void HinaPE::SmokeNativeSolver::_apply_pressure_forcePartial(float dt, SIM_VectorField *flow, const UT_JobInfo &info)
{

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
void HinaPE::SmokeNativeSolver::_enforce_boundaryPartial(SIM_RawField *Vaxis, int axis, const UT_JobInfo &info)
{
	// No-flux
	UT_VoxelArrayIteratorF vit;
	Vaxis->getPartialRange(vit, info);
	vit.setCompressOnExit(true);
	for (vit.rewind(); !vit.atEnd(); vit.advance())
	{
		switch (axis)
		{
			case 0:
			{
				int min_idx = 0;
				int max_idx = Vaxis->getXRes() - 1;
				if (vit.x() == min_idx || vit.x() == max_idx)
					vit.setValue(0.f);
			}
				break;
			case 1:
			{
				int min_idx = 0;
				int max_idx = Vaxis->getYRes() - 1;
				if (vit.y() == min_idx || vit.y() == max_idx)
					vit.setValue(0.f);
			}
				break;
			case 2:
			{
				int min_idx = 0;
				int max_idx = Vaxis->getZRes() - 1;
				if (vit.z() == min_idx || vit.z() == max_idx)
					vit.setValue(0.f);
			}
				break;
			default:
				throw std::runtime_error("Invalid axis");
		}
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
