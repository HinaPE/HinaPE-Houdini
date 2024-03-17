#ifndef HINAPE_SMOKE_H
#define HINAPE_SMOKE_H

/**
 * Grid Based Smoke Simulation
 *
 * NOT IMPLEMENTED YET
 */

#include <vector>
#include <UT/UT_Vector3.h>
#include <UT/UT_ParallelUtil.h>
#include <GAS/GAS_PBDSolve.h>
#include <SIM/SIM_ScalarField.h>
#include <SIM/SIM_FieldSampler.h>
#include <SIM/SIM_VectorField.h>
#include <SIM/SIM_FieldUtils.h>
#include "common/geometry.h"
namespace HinaPE
{
using Surface = HinaPE::ISurface<fpreal32, UT_Vector3, UT_Quaternion>;

struct SmokeNativeSolver
{
	void Init(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V);
	void Solve(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V);

protected:
	THREADED_METHOD5(SmokeNativeSolver, V->getXField()->shouldMultiThread(), init_smoke_source, float, dt, SIM_ScalarField*, D, SIM_ScalarField*, T, SIM_VectorField*, V, int, axis);
	THREADED_METHOD5(SmokeNativeSolver, V->getXField()->shouldMultiThread(), apply_external_force, float, dt, SIM_ScalarField*, D, SIM_ScalarField*, T, SIM_VectorField*, V, int, axis);
	THREADED_METHOD5(SmokeNativeSolver, V->getXField()->shouldMultiThread(), apply_viscosity, float, dt, SIM_ScalarField*, D, SIM_ScalarField*, T, SIM_VectorField*, V, int, axis);
	THREADED_METHOD5(SmokeNativeSolver, V->getXField()->shouldMultiThread(), apply_pressure, float, dt, SIM_ScalarField*, D, SIM_ScalarField*, T, SIM_VectorField*, V, int, axis);
	THREADED_METHOD5(SmokeNativeSolver, V->getXField()->shouldMultiThread(), advect, float, dt, SIM_ScalarField*, D, SIM_ScalarField*, T, SIM_VectorField*, V, int, axis);

	void init_smoke_sourcePartial(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, int axis, const UT_JobInfo &info);
	void apply_external_forcePartial(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, int axis, const UT_JobInfo &info);
	void apply_viscosityPartial(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, int axis, const UT_JobInfo &info);
	void apply_pressurePartial(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, int axis, const UT_JobInfo &info);
	void advectPartial(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, int axis, const UT_JobInfo &info);
};
}
#endif //HINAPE_SMOKE_H
