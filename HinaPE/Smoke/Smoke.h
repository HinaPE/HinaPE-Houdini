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
#include <SIM/SIM_VectorField.h>
#include <SIM/SIM_FieldUtils.h>
#include "common/geometry.h"
namespace HinaPE
{
using Surface = HinaPE::ISurface<fpreal32, UT_Vector3, UT_Quaternion>;

struct SmokeNativeSolver
{
	static void test(SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V);
	static void emit(SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V);
	static void apply_advection(float dt, SIM_ScalarField *SF, SIM_ScalarField *Co, const SIM_VectorField *V);

	THREADED_METHOD1(SmokeNativeSolver, true, apply_external_force, float, dt);
	THREADED_METHOD1(SmokeNativeSolver, true, apply_viscosity, float, dt);
	THREADED_METHOD1(SmokeNativeSolver, true, apply_pressure, float, dt);


	SIM_ScalarField *D;
	SIM_ScalarField *T;
	SIM_VectorField *V;

private:
	void apply_external_forcePartial(float dt, const UT_JobInfo &info);
	void apply_viscosityPartial(float dt, const UT_JobInfo &info);
	void apply_pressurePartial(float dt, const UT_JobInfo &info);
};
}
#endif //HINAPE_SMOKE_H
