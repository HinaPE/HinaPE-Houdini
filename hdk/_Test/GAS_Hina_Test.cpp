#include "GAS_Hina_Test.h"

#include <HinaPE/Smoke/FieldUtils.h>
#include <UT/UT_SparseMatrix.h>
#include <UT/UT_MatrixSolver.h>
#include <iostream>

class DiagonalPreconditioner
{
public:
	explicit DiagonalPreconditioner(const UT_SparseMatrix &A) :
			myID(new UT_Vector(0, A.getNumCols() - 1))
	{
		UT_Vector &id(*myID);
		A.extractDiagonal(id);
		// This asserts that A's diagonal is nonzero
		for (int i = 0; i < id.length(); ++i)
			id(i) = 1 / id(i);
	}
	void operator()(const UT_Vector &b, UT_Vector &x) const
	{
		x = b;
		x *= *myID;
	}
private:
	// inverse diagonal
	UT_SharedPtr<UT_Vector> myID;
};

class IterationTester
{
public:
	IterationTester() {}
	bool operator()(int iteration, const UT_Vector &r) const
	{
		if (iteration > myMaxIterations)
			return false;
		return (r.norm2() > myThreshold);
	}
private:
	int myMaxIterations = 100;
	fpreal myThreshold = 1e-2;
};

GAS_HINA_SUBSOLVER_IMPLEMENT(
		Test,
		true,
		false,
		ACTIVATE_GAS_VELOCITY \
        ACTIVATE_GAS_DENSITY \
        ACTIVATE_GAS_VELOCITY2 \
        ACTIVATE_GAS_DENSITY2 \
)
void GAS_Hina_Test::_init() {}
void GAS_Hina_Test::_makeEqual(const GAS_Hina_Test *src) {}
bool GAS_Hina_Test::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_ScalarField *D = getScalarField(obj, GAS_NAME_DENSITY);
	SIM_VectorField *V = getVectorField(obj, GAS_NAME_VELOCITY);
	SIM_ScalarField *D2 = getScalarField(obj, GAS_NAME_DENSITY2);
	SIM_VectorField *V2 = getVectorField(obj, GAS_NAME_VELOCITY2);

	if (!D || !V || !D2 || !V2)
		return false;

//	{
//		auto &Field1 = *D;
//		if (Field1.getVoxelSample() == SIM_SAMPLE_CENTER)
//		{
//			CubbyFlow::CellCenteredScalarGrid3Ptr Field2 = CubbyFlow::CellCenteredScalarGrid3::GetBuilder().MakeShared();
//			HinaPE::ToCubby(Field1, Field2);
//			HinaPE::print(Field1);
//			HinaPE::print(Field2);
//			HinaPE::match(Field1, Field2);
//
//			HinaPE::ToHDK(Field2, *D2);
//			std::cout << "isMatching D - D2: " << D->getField()->isMatching(D2->getField()) << std::endl;
//			std::cout << "isAligned D - D2: " << D->getField()->isAligned(D2->getField()) << std::endl;
//		} else if (Field1.getVoxelSample() == SIM_SAMPLE_CORNER)
//		{
//			CubbyFlow::VertexCenteredScalarGrid3Ptr Field2 = CubbyFlow::VertexCenteredScalarGrid3::GetBuilder().MakeShared();
//			HinaPE::ToCubby(Field1, Field2);
//			HinaPE::print(Field1);
//			HinaPE::print(Field2);
//			HinaPE::match(Field1, Field2);
//
//			HinaPE::ToHDK(Field2, *D2);
//			std::cout << "isMatching D - D2: " << D->getField()->isMatching(D2->getField()) << std::endl;
//			std::cout << "isAligned D - D2: " << D->getField()->isAligned(D2->getField()) << std::endl;
//		}
//	}
//
//	{
//		auto &Field1 = *V;
//		if (V->isFaceSampled())
//		{
//			CubbyFlow::FaceCenteredGrid3Ptr Field2 = CubbyFlow::FaceCenteredGrid3::GetBuilder().MakeShared();
//			HinaPE::ToCubby(Field1, Field2);
//			HinaPE::print(Field1);
//			HinaPE::print(Field2);
//			HinaPE::match(Field1, Field2);
//
//			std::cout << "isMatching V - V2: "
//					  << V->getXField()->isMatching(V2->getXField()) << ", "
//					  << V->getYField()->isMatching(V2->getYField()) << ", "
//					  << V->getZField()->isMatching(V2->getZField()) << std::endl;
//
//			std::cout << "isAligned V - V2: "
//					  << V->getXField()->isAligned(V2->getXField()) << ", "
//					  << V->getYField()->isAligned(V2->getYField()) << ", "
//					  << V->getZField()->isAligned(V2->getZField()) << std::endl;
//		}
//	}

	UT_SparseMatrix A(3, 3);
	UT_Vector x(0, 2);
	UT_Vector b(0, 2);
	A.addToElement(0, 0, 4);
	A.addToElement(0, 1, 1);
	A.addToElement(0, 2, 1);
	A.addToElement(1, 0, 1);
	A.addToElement(1, 1, 3);
	A.addToElement(1, 2, -1);
	A.addToElement(2, 0, 1);
	A.addToElement(2, 1, -1);
	A.addToElement(2, 2, 2);
	b(0) = 6;
	b(1) = 2;
	b(2) = -1;
	A.compile();
	A.printFull(std::cout);
	std::cout << "b: " << b << std::endl;

	UT_Functor2<void, const UT_Vector &, UT_Vector &> multiply_A(
			&A, &UT_SparseMatrix::multVec
	);
	const DiagonalPreconditioner diagonal_preconditioner(A);
	UT_Functor2<void, const UT_Vector &, UT_Vector &> preconditioner_A(diagonal_preconditioner);
	const IterationTester iteration_tester;
	UT_Functor2<bool, int, const UT_Vector &> keep_iterating(iteration_tester);
	UT_MatrixIterSolver::PCG(
			x, b,
			multiply_A,
			preconditioner_A,
			keep_iterating
	);
	std::cout << "x: " << x << std::endl;

	UT_Vector x2(0, 2);
	UT_SparseMatrixRowD sparseMatrix;
	sparseMatrix.buildFrom(A);
	int iter_out;
	fpreal error = sparseMatrix.solveConjugateGradient(x2, b, nullptr, 1e-2, 100, &iter_out);
	std::cout << "iter: " << iter_out << " x2: " << x2 << std::endl;

	return true;
}
