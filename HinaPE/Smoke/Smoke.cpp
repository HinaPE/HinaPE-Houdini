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
			if ((pos - center).length() < 0.3f)
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
			if ((pos - center).length() < 0.3f)
				vit.setValue(1.0f);
		}
	}
}
void HinaPE::SmokeNativeSolver::apply_external_forcePartial(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, int axis, const UT_JobInfo &info)
{

}
void HinaPE::SmokeNativeSolver::apply_viscosityPartial(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, int axis, const UT_JobInfo &info)
{

}
void HinaPE::SmokeNativeSolver::apply_pressurePartial(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, int axis, const UT_JobInfo &info)
{

}
void HinaPE::SmokeNativeSolver::advectPartial(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, int axis, const UT_JobInfo &info)
{
	UT_VoxelArrayIteratorF vit;
	T->getField()->getPartialRange(vit, info);
	vit.setCompressOnExit(true);

	for (vit.rewind(); !vit.atEnd(); vit.advance())
	{
		UT_Vector3 pos;
		D->indexToPos(vit.x(), vit.y(), vit.z(), pos);
		vit.setValue(D->getValue(pos));
	}
}
