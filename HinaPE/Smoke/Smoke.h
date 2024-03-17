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

struct SmokeNativeParam
{
	fpreal32 BUOYANCY_SMOKE_DENSITY_FACTOR = -0.000625f;
	fpreal32 BUOYANCY_SMOKE_TEMPERATURE_FACTOR = 5.f;
};

struct SmokeNativeSolver : public SmokeNativeParam
{
	void Init(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V);
	void Solve(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V);

protected:
	void emit(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V);
	void non_pressure(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V);
	void pressure(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V);
	void advect(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V);
	void advect_HDK(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V);
	void enforce_boundary(SIM_VectorField *V);

private:
	THREADED_METHOD1(SmokeNativeSolver, D->getField()->shouldMultiThread(), _emit_density, SIM_ScalarField*, D);
	THREADED_METHOD1(SmokeNativeSolver, T->getField()->shouldMultiThread(), _emit_temperature, SIM_ScalarField*, T);
	THREADED_METHOD4(SmokeNativeSolver, flow->getXField()->shouldMultiThread(), _apply_buoyancy_force, float, dt, SIM_ScalarField*, D, SIM_ScalarField*, T, SIM_VectorField*, flow);
	THREADED_METHOD2(SmokeNativeSolver, flow->getXField()->shouldMultiThread(), _apply_pressure_force, float, dt, SIM_VectorField*, flow);
	THREADED_METHOD4(SmokeNativeSolver, flow->getXField()->shouldMultiThread(), _advect_field, float, dt, SIM_RawField*, output, const SIM_RawField*, input, SIM_VectorField*, flow);
	THREADED_METHOD2(SmokeNativeSolver, Vaxis->shouldMultiThread(), _enforce_boundary, SIM_RawField*, Vaxis, int, axis);

	void _emit_densityPartial(SIM_ScalarField *D, const UT_JobInfo &info);
	void _emit_temperaturePartial(SIM_ScalarField *T, const UT_JobInfo &info);
	void _apply_buoyancy_forcePartial(float dt, SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *flow, const UT_JobInfo &info);
	void _apply_pressure_forcePartial(float dt, SIM_VectorField *flow, const UT_JobInfo &info);
	void _advect_fieldPartial(float dt, SIM_RawField *output, const SIM_RawField *input, SIM_VectorField *flow, const UT_JobInfo &info);
	void _enforce_boundaryPartial(SIM_RawField *Vaxis, int axis, const UT_JobInfo &info);

private:
	auto _back_trace(float dt, const UT_Vector3 &pt, const SIM_VectorField *V) -> UT_Vector3;
};
}
#endif //HINAPE_SMOKE_H
