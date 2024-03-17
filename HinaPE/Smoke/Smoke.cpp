#include "Smoke.h"
#include <UT/UT_MatrixSolver.h>
#include <UT/UT_ParallelUtil.h>
#include <UT/UT_ThreadedAlgorithm.h>
#include <UT/UT_Interrupt.h>
#include <memory>

void HinaPE::SmokeNativeSolver::Init(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V)
{
	init_smoke_source(dt, D, T, V, 1);
}
void HinaPE::SmokeNativeSolver::Solve(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V)
{
	for (auto axis: {0, 1, 2})
	{
		apply_external_force(dt, D, T, V, axis);
		apply_viscosity(dt, D, T, V, axis);
		apply_pressure(dt, D, T, V, axis);
		advect(dt, D, T, V, 0);
	}
}

void HinaPE::SmokeNativeSolver::init_smoke_sourcePartial(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, int axis, const UT_JobInfo &info)
{
	auto center = SIMfieldUtilsGetCenter(D);

	{
		UT_VoxelArrayIteratorF vit;
		D->getField()->getPartialRange(vit, info);
		vit.setCompressOnExit(true);
		for (vit.rewind(); !vit.atEnd(); vit.advance())
		{
			UT_Vector3 pos;
			D->indexToPos(vit.x(), vit.y(), vit.z(), pos);
			if ((pos - center).length() < 0.5f)
				vit.setValue(1.0f);
		}
	}

	{
		UT_VoxelArrayIteratorF vit;
		T->getField()->getPartialRange(vit, info);
		vit.setCompressOnExit(true);
		for (vit.rewind(); !vit.atEnd(); vit.advance())
		{
			UT_Vector3 pos;
			T->indexToPos(vit.x(), vit.y(), vit.z(), pos);
			if ((pos - center).length() < 0.5f)
				vit.setValue(.1f);
		}
	}
}
void HinaPE::SmokeNativeSolver::apply_external_forcePartial(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, int axis, const UT_JobInfo &info)
{
	_compute_buoyancy(dt, D, T, V, axis, info);
}
void HinaPE::SmokeNativeSolver::apply_viscosityPartial(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, int axis, const UT_JobInfo &info)
{

}
void HinaPE::SmokeNativeSolver::apply_pressurePartial(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, int axis, const UT_JobInfo &info)
{

}
void HinaPE::SmokeNativeSolver::advectPartial(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, int axis, const UT_JobInfo &info)
{

}
void HinaPE::SmokeNativeSolver::_compute_buoyancy(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, int axis, const UT_JobInfo &info)
{
	UT_VoxelArrayIteratorF vit;
	V->getField(axis)->getPartialRange(vit, info);
	vit.setCompressOnExit(true);

	for (vit.rewind(); !vit.atEnd(); vit.advance())
	{
		UT_Vector3 vel_pos;
		V->indexToPos(axis, vit.x(), vit.y(), vit.z(), vel_pos);

		fpreal32 den = D->getValue(vel_pos);
		fpreal32 tem = T->getValue(vel_pos);

		if (den < std::numeric_limits<fpreal32>::epsilon() || tem < std::numeric_limits<fpreal32>::epsilon())
			continue;
		const fpreal32 f_buoyancy = BUOYANCY_SMOKE_DENSITY_FACTOR * den + BUOYANCY_SMOKE_TEMPERATURE_FACTOR * tem;
		vit.setValue(vit.getValue() + dt + f_buoyancy);
	}
}
