#ifndef HINAPE_ADVECTIONSOLVER_H
#define HINAPE_ADVECTIONSOLVER_H

#include <SIM/SIM_RawField.h>

namespace HinaPE
{
struct AdvectionSolver
{
	THREADED_METHOD6(AdvectionSolver, OutField->shouldMultiThread(), _advect, float, dt, SIM_RawField *, OutField, const SIM_RawField *, InField, const SIM_RawField *, Flow_X, const SIM_RawField *, Flow_Y, const SIM_RawField *, Flow_Z);
	void _advectPartial(float dt, SIM_RawField *OutField, const SIM_RawField *InField, const SIM_RawField *Flow_X, const SIM_RawField *Flow_Y, const SIM_RawField *Flow_Z, const UT_JobInfo &info);
	UT_Vector3 _back_trace(float dt, const UT_Vector3 &pt, const SIM_RawField *Flow_X, const SIM_RawField *Flow_Y, const SIM_RawField *Flow_Z);
};
}

#endif //HINAPE_ADVECTIONSOLVER_H
