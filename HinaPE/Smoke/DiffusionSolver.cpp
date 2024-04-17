#include "DiffusionSolver.h"

#include <UT/UT_SparseMatrix.h>

void HinaPE::DiffusionSolver::solve(SIM_RawField *IO_Field, const SIM_RawField *CollisionSDF)
{
	size_t size = IO_Field->getVoxelRes().x() * IO_Field->getVoxelRes().y() * IO_Field->getVoxelRes().z();
	UT_SparseMatrix A(size, size);
	UT_Vector x(0, size);
	UT_Vector b(0, size);

	SIM_RawField Marker;
	_build_marker(&Marker, IO_Field, CollisionSDF);
	_build_matrix(A, IO_Field, &Marker);
	_build_vector(b, IO_Field, &Marker);

	A.compile();
	x = b;
	UT_SparseMatrixRowD sparseMatrix;
	sparseMatrix.buildFrom(A);
	sparseMatrix.solveConjugateGradient(x, b, nullptr);

	_assign_field(IO_Field, x);
}
void HinaPE::DiffusionSolver::_build_markerPartial(SIM_RawField *OUT_Marker, const SIM_RawField *IN_Field, const SIM_RawField *CollisionSDF, const UT_JobInfo &info)
{
	UT_VoxelArrayIteratorF vit;
	IN_Field->getPartialRange(vit, info);
	vit.setCompressOnExit(true);
	for (vit.rewind(); !vit.atEnd(); vit.advance())
	{

	}
}
void HinaPE::DiffusionSolver::_build_matrixPartial(UT_SparseMatrix &OUT_Matrix, const SIM_RawField *IN_Field, const SIM_RawField * IN_Marker, const UT_JobInfo &info)
{
	UT_VoxelArrayIteratorF vit;
	IN_Field->getPartialRange(vit, info);
	vit.setCompressOnExit(true);
	for (vit.rewind(); !vit.atEnd(); vit.advance())
	{

	}
}
void HinaPE::DiffusionSolver::_build_vectorPartial(UT_Vector &OUT_Vector, const SIM_RawField *IN_Field, const SIM_RawField * IN_Marker, const UT_JobInfo &info)
{
	UT_VoxelArrayIteratorF vit;
	IN_Field->getPartialRange(vit, info);
	vit.setCompressOnExit(true);
	for (vit.rewind(); !vit.atEnd(); vit.advance())
	{

	}
}
void HinaPE::DiffusionSolver::_assign_fieldPartial(SIM_RawField *OUT_Field, const UT_Vector &IN_Vector, const UT_JobInfo &info)
{

}
