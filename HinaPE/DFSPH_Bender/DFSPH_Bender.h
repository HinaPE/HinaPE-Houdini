#ifndef HINAPE_DFSPH_BENDER_H
#define HINAPE_DFSPH_BENDER_H

/**
 * DFSPH implementation, Bender Boundary
 * https://github.com/InteractiveComputerGraphics/SPlisHSPlasH
 */

#include "common/fluid.h"
#include "common/kernels.h"
#include "common/neighbors.h"
#include "common/geometry.h"
#include <vector>
#include <UT/UT_Vector3.h>
namespace HinaPE
{
using real = float;
using Vector = UT_Vector3T<real>;
using Kernel = Cubic<real, Vector>;
using ScalarArrayCPU = std::vector<real>;
using VectorArrayCPU = std::vector<Vector>;
using FluidCPU = IFluid<real, Vector, ScalarArrayCPU, VectorArrayCPU>;
using BenderBoundaryCPU = IBenderBoundary<real, Vector, ScalarArrayCPU, VectorArrayCPU>;
using NeighborBuilder = NeighborBuilderGPU<real, Vector, ScalarArrayCPU, VectorArrayCPU>;

struct DFSPH_BenderFluidCPU : public FluidCPU
{
	ScalarArrayCPU factor;
	ScalarArrayCPU k;
	ScalarArrayCPU density_adv;
};

struct DFSPH_BenderSParam
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

struct DFSPH_BenderSolver : public DFSPH_BenderSParam
{
	DFSPH_BenderSolver(real, Vector);
	void Solve(real dt);
	std::shared_ptr<DFSPH_BenderFluidCPU> Fluid;
	std::vector<std::shared_ptr<BenderBoundaryCPU>> Boundaries;

private:
	NeighborBuilder NeighborBuilder;
	Vector MaxBound;
	bool VolumeInited;
};
}

#endif //HINAPE_DFSPH_BENDER_H
