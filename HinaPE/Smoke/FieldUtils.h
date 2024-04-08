#ifndef HINAPE_FIELDUTILS_H
#define HINAPE_FIELDUTILS_H

#include <UT/UT_Vector3.h>
#include <SIM/SIM_RawField.h>

#include <CUDA_CubbyFlow/Core/Solver/Grid/GridBackwardEulerDiffusionSolver3.hpp>
#include <CUDA_CubbyFlow/Core/Grid/CellCenteredScalarGrid.hpp>
#include <CUDA_CubbyFlow/Core/Grid/VertexCenteredScalarGrid.hpp>

namespace HinaPE
{
using real = float;
using Vector = UT_Vector3T<real>;

CubbyFlow::ScalarGrid3Ptr ToCubby(const SIM_RawField &Field);
SIM_RawField ToHDK(const CubbyFlow::ScalarGrid3Ptr &Field);

struct BackwardDiffusionSolver
{
	BackwardDiffusionSolver();
	void Solve(const SIM_RawField &Input, SIM_RawField &Marker, SIM_RawField &Output);
	real stiffness;
	real dt;

	CubbyFlow::GridBackwardEulerDiffusionSolver3Ptr solver;
};

bool match(const SIM_RawField &Field1, const CubbyFlow::ScalarGrid3 &Field2);
void print(const SIM_RawField &Field);
void print(const CubbyFlow::ScalarGrid3 &Field);
}

#endif //HINAPE_FIELDUTILS_H
