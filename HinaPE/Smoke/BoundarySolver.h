#ifndef HINAPE_BOUNDARYSOLVER_H
#define HINAPE_BOUNDARYSOLVER_H

#include <SIM/SIM_RawField.h>

namespace HinaPE
{
struct BoundaryParam
{
	float DEPTH;
	float FRICTION;
	bool CLOSE_LEFT;
	bool CLOSE_RIGHT;
	bool CLOSE_TOP;
	bool CLOSE_BOTTOM;
	bool CLOSE_FRONT;
	bool CLOSE_BACK;
};

struct BoundarySolver : public BoundaryParam
{
	THREADED_METHOD4(BoundarySolver, OUT_Marker->shouldMultiThread(), _build_marker, SIM_RawField *, OUT_Marker, SIM_RawField *, OUT_V, const SIM_RawField *, CollisionSDF, const UT_Axis3::axis&, AXIS);
	void _build_markerPartial(SIM_RawField *OUT_Marker, SIM_RawField *OUT_V, const SIM_RawField *CollisionSDF, const UT_Axis3::axis &AXIS, const UT_JobInfo &info);

	THREADED_METHOD3(BoundarySolver, OUT_V->shouldMultiThread(), _extrapolate, SIM_RawField *, OUT_V, SIM_RawField *, OUT_Marker, const SIM_RawField *, IN_Marker);
	void _extrapolatePartial(SIM_RawField *OUT_V, SIM_RawField *OUT_Marker, const SIM_RawField *IN_Marker, const UT_JobInfo &info);

	THREADED_METHOD6(BoundarySolver, OUT_V->shouldMultiThread(), _fractional, SIM_RawField *, OUT_V, SIM_RawField *, IN_V_X, SIM_RawField *, IN_V_Y, SIM_RawField *, IN_V_Z, const SIM_RawField *, CollisionSDF, const UT_Axis3::axis&, AXIS);
	void _fractionalPartial(SIM_RawField *OUT_V, const SIM_RawField *IN_V_X, const SIM_RawField *IN_V_Y, const SIM_RawField *IN_V_Z, const SIM_RawField *CollisionSDF, const UT_Axis3::axis &AXIS, const UT_JobInfo &info);
	UT_Vector3 _project_and_apply_friction(const UT_Vector3 &vel, const UT_Vector3 normal);
	void _enforce_boundary(SIM_RawField *OUT_V_X, SIM_RawField *OUT_V_Y, SIM_RawField *OUT_V_Z);
};
}

#endif //HINAPE_BOUNDARYSOLVER_H
