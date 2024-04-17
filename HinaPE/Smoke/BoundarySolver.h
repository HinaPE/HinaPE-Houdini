#ifndef HINAPE_BOUNDARYSOLVER_H
#define HINAPE_BOUNDARYSOLVER_H

#include <SIM/SIM_RawField.h>

namespace HinaPE
{
struct BoundarySolver
{
	THREADED_METHOD4(BoundarySolver, V_X->shouldMultiThread(), _fractional, SIM_RawField *, V_X, SIM_RawField *, V_Y, SIM_RawField *, V_Z, const SIM_RawField *, CollisionSDF);
	void _fractionalPartial(SIM_RawField *V_X, SIM_RawField *V_Y, SIM_RawField *V_Z, const SIM_RawField *CollisionSDF, const UT_JobInfo &info);

	THREADED_METHOD4(BoundarySolver, V_X->shouldMultiThread(), _blocked, SIM_RawField *, V_X, SIM_RawField *, V_Y, SIM_RawField *, V_Z, const SIM_RawField *, CollisionSDF);
	void _blockedPartial(SIM_RawField *V_X, SIM_RawField *V_Y, SIM_RawField *V_Z, const SIM_RawField *CollisionSDF, const UT_JobInfo &info);
};
}

#endif //HINAPE_BOUNDARYSOLVER_H
