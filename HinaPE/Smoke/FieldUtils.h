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

void ToCubby(const SIM_ScalarField &Field, CubbyFlow::CellCenteredScalarGrid3Ptr &Output);
void ToCubby(const SIM_ScalarField &Field, CubbyFlow::VertexCenteredScalarGrid3Ptr &Output);
void ToCubby(const SIM_VectorField &Field, CubbyFlow::CellCenteredVectorGrid3Ptr &Output);
void ToCubby(const SIM_VectorField &Field, CubbyFlow::VertexCenteredVectorGrid3Ptr &Output);
void ToCubby(const SIM_VectorField &Field, CubbyFlow::FaceCenteredGrid3Ptr &Output);
void ToHDK(const CubbyFlow::ScalarGrid3Ptr &Field, SIM_ScalarField &Output);
void ToHDK(const CubbyFlow::VectorGrid3Ptr &Field, SIM_VectorField &Output);

void match(const SIM_ScalarField &FieldHDK, const CubbyFlow::CellCenteredScalarGrid3Ptr &FieldCubby);
void match(const SIM_ScalarField &FieldHDK, const CubbyFlow::VertexCenteredScalarGrid3Ptr &FieldCubby);
void match(const SIM_VectorField &FieldHDK, const CubbyFlow::CellCenteredVectorGrid3Ptr &FieldCubby);
void match(const SIM_VectorField &FieldHDK, const CubbyFlow::VertexCenteredVectorGrid3Ptr &FieldCubby);
void match(const SIM_VectorField &FieldHDK, const CubbyFlow::FaceCenteredGrid3Ptr &FieldCubby);

void print(const SIM_ScalarField &Field);
void print(const SIM_VectorField &Field);
void print(const SIM_RawField &Field);
void print(const CubbyFlow::CellCenteredScalarGrid3Ptr &Field);
void print(const CubbyFlow::VertexCenteredScalarGrid3Ptr &Field);
void print(const CubbyFlow::CellCenteredVectorGrid3Ptr &Field);
void print(const CubbyFlow::VertexCenteredVectorGrid3Ptr &Field);
void print(const CubbyFlow::FaceCenteredGrid3Ptr &Field);
}

#endif //HINAPE_FIELDUTILS_H
