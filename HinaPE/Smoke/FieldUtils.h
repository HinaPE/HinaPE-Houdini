#ifndef HINAPE_FIELDUTILS_H
#define HINAPE_FIELDUTILS_H

#include <UT/UT_Vector3.h>
#include <SIM/SIM_RawField.h>
#include <SIM/SIM_ScalarField.h>
#include <SIM/SIM_VectorField.h>

#include <CUDA_CubbyFlow/Core/Solver/Grid/GridBackwardEulerDiffusionSolver3.hpp>
#include <CUDA_CubbyFlow/Core/Grid/CellCenteredScalarGrid.hpp>
#include <CUDA_CubbyFlow/Core/Grid/VertexCenteredScalarGrid.hpp>
#include <CUDA_CubbyFlow/Core/Grid/CellCenteredVectorGrid.hpp>
#include <CUDA_CubbyFlow/Core/Grid/VertexCenteredVectorGrid.hpp>
#include <CUDA_CubbyFlow/Core/Grid/FaceCenteredGrid.hpp>

namespace HinaPE
{
using real = float;
using Vector = UT_Vector3T<real>;

void ToCubby(const SIM_ScalarField &Field, CubbyFlow::ScalarGrid3Ptr &Output);
void ToCubby(const SIM_VectorField &Field, CubbyFlow::VectorGrid3Ptr &Output);
void ToHDK(const CubbyFlow::ScalarGrid3Ptr &Field, SIM_ScalarField &Output);
void ToHDK(const CubbyFlow::VectorGrid3Ptr &Field, SIM_VectorField &Output);

void match(const SIM_ScalarField &FieldHDK, const CubbyFlow::CellCenteredScalarGrid3Ptr &FieldCubby);
void match(const SIM_VectorField &FieldHDK, const CubbyFlow::FaceCenteredGrid3Ptr &FieldCubby);

void print(const SIM_ScalarField &Field);
void print(const SIM_VectorField &Field);
void print(const SIM_RawField &Field);


















CubbyFlow::ScalarGrid3Ptr ToCubby(const SIM_RawField &Field); // @deprecated
SIM_RawField ToHDK(const CubbyFlow::ScalarGrid3Ptr &Field); // @deprecated

struct BackwardDiffusionSolver
{
	BackwardDiffusionSolver();
	void Solve(const SIM_RawField &Input, SIM_RawField &Marker, SIM_RawField &Output);
	real stiffness;
	real dt;

	CubbyFlow::GridBackwardEulerDiffusionSolver3Ptr solver;
};

bool match(const SIM_RawField &Field1, const CubbyFlow::ScalarGrid3 &Field2);
void print(const CubbyFlow::ScalarGrid3 &Field);
}

#endif //HINAPE_FIELDUTILS_H
