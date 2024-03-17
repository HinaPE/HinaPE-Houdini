#include "Smoke.h"
#include <UT/UT_MatrixSolver.h>
#include <UT/UT_ParallelUtil.h>
#include <UT/UT_ThreadedAlgorithm.h>
#include <UT/UT_Interrupt.h>
#include <memory>

void HinaPE::SmokeNativeSolver::Init(SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, float dt)
{
	init_smoke_source(D, T);
}
void HinaPE::SmokeNativeSolver::Solve(SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, float dt)
{
	apply_external_force(dt, D, T, V);
	apply_viscosity(dt, D, T, V);
	apply_pressure(dt, D, T, V);
	advect(dt, D, T, V);
}

void HinaPE::SmokeNativeSolver::init_smoke_sourcePartial(SIM_ScalarField *D, SIM_ScalarField *T, const UT_JobInfo &info)
{
	UT_Interrupt *boss = UTgetInterrupt();
	if (boss->opInterrupt())
		return;

//	auto center = D->getOrig() + 0.5 * D->getSize();
	auto center = SIMfieldUtilsGetCenter(D);

	{
		UT_VoxelArrayIteratorF vit;
		vit.setArray(D->getField()->fieldNC());
		vit.setCompressOnExit(true);
		vit.setPartialRange(info.job(), info.numJobs());
		for (vit.rewind(); !vit.atEnd(); vit.advance())
		{
			UT_Vector3 pos;
			D->indexToPos(vit.x(), vit.y(), vit.z(), pos);
			if ((pos - center).length() < 0.3)
				vit.setValue(1.0f);
		}
	}
	{
		UT_VoxelArrayIteratorF vit;
		vit.setArray(T->getField()->fieldNC());
		vit.setCompressOnExit(true);
		vit.setPartialRange(info.job(), info.numJobs());
		for (vit.rewind(); !vit.atEnd(); vit.advance())
		{
			UT_Vector3 pos;
			T->indexToPos(vit.x(), vit.y(), vit.z(), pos);
			if ((pos - center).length() < 0.3)
				vit.setValue(1.0f);
		}
	}
}
void HinaPE::SmokeNativeSolver::apply_external_forcePartial(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, const UT_JobInfo &info)
{

}
void HinaPE::SmokeNativeSolver::apply_viscosityPartial(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, const UT_JobInfo &info)
{

}
void HinaPE::SmokeNativeSolver::apply_pressurePartial(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, const UT_JobInfo &info)
{

}
void HinaPE::SmokeNativeSolver::advectPartial(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, const UT_JobInfo &info)
{

}
void HinaPE::SmokeNativeSolver::_compute_buoyancy(float dt, SIM_ScalarField *T, SIM_VectorField *V)
{

}
