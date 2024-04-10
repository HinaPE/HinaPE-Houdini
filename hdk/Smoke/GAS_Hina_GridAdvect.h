#ifndef HINAPE__GAS_HINA_GRIDADVECT_H
#define HINAPE__GAS_HINA_GRIDADVECT_H

#include <SIM_Hina_Generator.h>

/*
 * This is the same as node [GAS_Advect] in Houdini
 * use [GAS_Advect] instead
 */

GAS_HINA_SUBSOLVER_CLASS(
		GridAdvect,

		THREADED_METHOD6(GAS_Hina_GridBoundarySolver, OutField->shouldMultiThread(), _advect, float, dt, SIM_RawField * , OutField, const SIM_RawField *, InField, const SIM_RawField *, Flow_X, const SIM_RawField *, Flow_Y, const SIM_RawField *, Flow_Z);
				void _advectPartial(float dt, SIM_RawField *OutField, const SIM_RawField *InField, const SIM_RawField *Flow_X, const SIM_RawField *Flow_Y, const SIM_RawField *Flow_Z, const UT_JobInfo &info);
				UT_Vector3 _back_trace(float dt, const UT_Vector3 &pt, const SIM_RawField *Flow_X, const SIM_RawField *Flow_Y, const SIM_RawField *Flow_Z);
)

#endif //HINAPE__GAS_HINA_GRIDADVECT_H
