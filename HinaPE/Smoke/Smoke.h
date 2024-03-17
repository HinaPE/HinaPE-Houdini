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
	void Init(SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, float dt);
	void Solve(SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, float dt);

protected:
	THREADED_METHOD2(SmokeNativeSolver, true, init_smoke_source, SIM_ScalarField*, D, SIM_ScalarField*, T);
	THREADED_METHOD4(SmokeNativeSolver, true, apply_external_force, float, dt, SIM_ScalarField*, D, SIM_ScalarField*, T, SIM_VectorField*, V);
	THREADED_METHOD4(SmokeNativeSolver, true, apply_viscosity, float, dt, SIM_ScalarField*, D, SIM_ScalarField*, T, SIM_VectorField*, V);
	THREADED_METHOD4(SmokeNativeSolver, true, apply_pressure, float, dt, SIM_ScalarField*, D, SIM_ScalarField*, T, SIM_VectorField*, V);
	THREADED_METHOD4(SmokeNativeSolver, true, advect, float, dt, SIM_ScalarField*, D, SIM_ScalarField*, T, SIM_VectorField*, V);

	void init_smoke_sourcePartial(SIM_ScalarField *D, SIM_ScalarField *T, const UT_JobInfo &info);
	void apply_external_forcePartial(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, const UT_JobInfo &info);
	void apply_viscosityPartial(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, const UT_JobInfo &info);
	void apply_pressurePartial(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, const UT_JobInfo &info);
	void advectPartial(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V, const UT_JobInfo &info);

private:
	void _compute_buoyancy(float dt, SIM_ScalarField *T, SIM_VectorField *V);
};
}
#endif //HINAPE_SMOKE_H
