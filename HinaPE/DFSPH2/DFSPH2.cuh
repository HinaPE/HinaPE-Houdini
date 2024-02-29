#ifndef HINAPE_DFSPH2_H
#define HINAPE_DFSPH2_H

/**
 * DFSPH implementation, SPH_Taichi version, CUDA backend
 * https://github.com/erizmr/SPH_Taichi
 */

#include "common/fluid.h"
#include "common/kernels.h"
#include "common/neighbors.h"
#include <thrust/universal_vector.h>
#include <UT/UT_Vector3.h>

namespace HinaPE
{
using real = float;
using Vector = UT_Vector3T<real>;
using Kernel = Cubic<real, Vector>;
using ScalarArrayGPU = thrust::universal_vector<real>;
using VectorArrayGPU = thrust::universal_vector<Vector>;
using FluidGPU = IFluid<real, Vector, ScalarArrayGPU, VectorArrayGPU>;
using AkinciBoundaryGPU = IAkinciBoundary<real, Vector, ScalarArrayGPU, VectorArrayGPU>;
using NeighborBuilder = NeighborBuilderGPU<real, Vector, ScalarArrayGPU, VectorArrayGPU>;
using FluidEmitter = IFluidEmitter<real, Vector, ScalarArrayGPU, VectorArrayGPU>;

struct DFSPH2FluidGPU : public FluidGPU
{
	ScalarArrayGPU factor;
	ScalarArrayGPU k;
	ScalarArrayGPU density_adv;
};

struct DFSPPH2Param
{
	real FLUID_REST_DENSITY = 1000.0f;
	std::vector<real> BOUNDARY_REST_DENSITY;
	real FLUID_PARTICLE_RADIUS = 0.01;
	real FLUID_SURFACE_TENSION = 0.01;
	real FLUID_VISCOSITY = 0.01;
	real BOUNDARY_VISCOSITY = 0;
	Vector GRAVITY = Vector(0, -9.8, 0);
	bool TOP_OPEN = true;
};

struct DFSPH2Solver : public DFSPPH2Param
{
	DFSPH2Solver(real, Vector);
	void Solve(real dt);
	std::shared_ptr<DFSPH2FluidGPU> Fluid;
	std::vector<std::shared_ptr<AkinciBoundaryGPU>> Boundaries;

protected:
	NeighborBuilder NeighborBuilder;
	Vector MaxBound;
};
}

#endif //HINAPE_DFSPH2_H
