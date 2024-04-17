#ifndef HINAPE_DIFFUSIONSOLVER_H
#define HINAPE_DIFFUSIONSOLVER_H

#include <SIM/SIM_RawField.h>
#include <UT/UT_SparseMatrix.h>

namespace HinaPE
{
struct DiffusionParam
{
	float DIFFUSION = 0.0f;
};

struct DiffusionSolver : public DiffusionParam
{
	void solve(SIM_RawField *IO_Field, const SIM_RawField *CollisionSDF);

private:
	THREADED_METHOD3(DiffusionSolver, OUT_Marker->shouldMultiThread(), _build_marker, SIM_RawField *, OUT_Marker, const SIM_RawField *, IN_Field, const SIM_RawField *, CollisionSDF);
	void _build_markerPartial(SIM_RawField *OUT_Marker, const SIM_RawField *IN_Field, const SIM_RawField *CollisionSDF, const UT_JobInfo &info);

	THREADED_METHOD3(DiffusionSolver, IN_Field->shouldMultiThread(), _build_matrix, UT_SparseMatrix &, OUT_Matrix, const SIM_RawField *, IN_Field, const SIM_RawField *, IN_Marker);
	void _build_matrixPartial(UT_SparseMatrix &OUT_Matrix, const SIM_RawField *IN_Field, const SIM_RawField *IN_Marker, const UT_JobInfo &info);

	THREADED_METHOD3(DiffusionSolver, IN_Field->shouldMultiThread(), _build_vector, UT_Vector &, OUT_Vector, const SIM_RawField *, IN_Field, const SIM_RawField *, IN_Marker);
	void _build_vectorPartial(UT_Vector &OUT_Vector, const SIM_RawField *IN_Field, const SIM_RawField *IN_Marker, const UT_JobInfo &info);

	THREADED_METHOD2(DiffusionSolver, OUT_Field->shouldMultiThread(), _assign_field, SIM_RawField *, OUT_Field, const UT_Vector &, IN_Vector);
	void _assign_fieldPartial(SIM_RawField *OUT_Field, const UT_Vector &IN_Vector, const UT_JobInfo &info);
};
}

#endif //HINAPE_DIFFUSIONSOLVER_H
