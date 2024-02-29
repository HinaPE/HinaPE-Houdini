#ifndef HINAPE_DFSPH1_H
#define HINAPE_DFSPH1_H

/**
 * DFSPH implementation, SPH_Taichi version
 * https://github.com/erizmr/SPH_Taichi
 */

#include "common/fluid.h"
#include "common/kernels.h"
#include "common/neighbors.h"
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
using AkinciBoundaryCPU = IAkinciBoundary<real, Vector, ScalarArrayCPU, VectorArrayCPU>;
using NeighborBuilder = NeighborBuilderGPU<real, Vector, ScalarArrayCPU, VectorArrayCPU>;
using FluidEmitter = IFluidEmitter<real, Vector, ScalarArrayCPU, VectorArrayCPU>;


struct DFSPH1FluidCPU : public FluidCPU
{
	ScalarArrayCPU factor;
	ScalarArrayCPU k;
	ScalarArrayCPU density_adv;
};

struct DFSPPH1Param
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

struct DFSPH1Solver : public DFSPPH1Param
{
	DFSPH1Solver(real, Vector);
	void Solve(real dt);
	std::shared_ptr<DFSPH1FluidCPU> Fluid;
	std::vector<std::shared_ptr<AkinciBoundaryCPU>> Boundaries;

protected:
	void build_neighbors();
	void compute_density();
	void compute_factor();
	void divergence_solve(real dt);
	void non_pressure_force();
	void predict_velocity(real dt);
	void pressure_solve(real dt);
	void advect(real dt);
	void enforce_boundary();

private:
	void _for_each_fluid_particle(const std::function<void(size_t, Vector)> &);
	void _for_each_neighbor_fluid(size_t, const std::function<void(size_t, Vector)> &);
	void _for_each_neighbor_boundaries(size_t, const std::function<void(size_t, Vector, size_t)> &);
	void _resize();
	NeighborBuilder NeighborBuilder;
	Vector MaxBound;
	bool VolumeInited;

private:
	void _compute_akinci_volume();
	void _compute_density_change();
	void _compute_density_adv(real dt);
	real _compute_density_error(const real offset);
	real _divergence_solver_iteration(real dt);
	real _pressure_solve_iteration(real dt);
	void _divergence_solver_iteration_kernel(real dt);
	void _pressure_solve_iteration_kernel(real dt);
};
}

#endif //HINAPE_DFSPH1_H
