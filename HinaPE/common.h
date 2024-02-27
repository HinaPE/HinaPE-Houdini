#ifndef HINAPE_COMMON_H
#define HINAPE_COMMON_H

#include <vector>
#include <array>
#include <thrust/universal_vector.h>
#include <UT/UT_Vector3.h>

namespace HinaPE
{
using real = float;
using Vector = UT_Vector3T<real>;
using CPUVectorArray = std::vector<Vector>;
using GPUVectorArray = thrust::universal_vector<Vector>;
using CPUScalarArray = std::vector<real>;
using GPUScalarArray = thrust::universal_vector<real>;

constexpr real FluidRestDensity = 1000.f;
}

#endif //HINAPE_COMMON_H
