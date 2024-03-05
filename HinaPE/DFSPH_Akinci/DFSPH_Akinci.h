#ifndef HINAPE_DFSPH_AKINCI_H
#define HINAPE_DFSPH_AKINCI_H

/**
 * DFSPH implementation, Akinci Boundary
 * https://github.com/erizmr/SPH_Taichi
 */

#include "common/particles.h"
#include "common/emitter.h"
#include "common/kernels.h"
#include "common/neighbors.h"
#include <vector>
#include <UT/UT_Vector3.h>
#include <UT/UT_Matrix4.h>
#include <UT/UT_Quaternion.h>
namespace HinaPE
{
using real = float;
using Vector = UT_Vector3T<real>;
using Kernel = ICubic<real, Vector>;
using ScalarArrayCPU = std::vector<real>;
using VectorArrayCPU = std::vector<Vector>;
using FluidCPU = IFluid<real, Vector, ScalarArrayCPU, VectorArrayCPU>;
using AkinciBoundaryCPU = IAkinciBoundary<real, Vector, ScalarArrayCPU, VectorArrayCPU>;
using NeighborBuilder = NeighborBuilderGPU<real, Vector, ScalarArrayCPU, VectorArrayCPU>;
using FluidEmitter = IFluidEmitter<real, Vector, ScalarArrayCPU, VectorArrayCPU>;

struct AkinciBoundary : public AkinciBoundaryCPU
{
	VectorArrayCPU x_init;
	UT_DMatrix4 xform; // update from outside. if Boundary is static, [xform] is the center of mass of the boundary, and keep the same all the time // if Boundary is dynamic, [xform] is updated by RigidBody simulator (update from outside)
};

struct DFSPH_AkinciFluid : public FluidCPU
{
	ScalarArrayCPU factor;
	ScalarArrayCPU k;
	ScalarArrayCPU density_adv;
};

struct DFSPH_AkinciParam
{
	real FLUID_REST_DENSITY = 1000.0f;
	real FLUID_PARTICLE_RADIUS = 0.01;
	real FLUID_SURFACE_TENSION = 0.01;
	real FLUID_VISCOSITY = 0.01;
	real BOUNDARY_VISCOSITY = 0;
	Vector GRAVITY = Vector(0, -9.8, 0);
	bool TOP_OPEN = true;

	std::vector<real> BOUNDARY_REST_DENSITY;
	std::vector<bool> BOUNDARY_DYNAMICS;
};

struct DFSPH_AkinciSolver : public DFSPH_AkinciParam
{
	DFSPH_AkinciSolver(real, Vector);
	virtual void Solve(real dt);
	std::shared_ptr<DFSPH_AkinciFluid> Fluid;
	std::vector<std::shared_ptr<AkinciBoundary>> Boundaries;

protected:
	void resize();
	void update_akinci_boundaries();
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
	NeighborBuilder NeighborBuilder;
	Vector MaxBound;
	bool NeighborBuilderInited;
	bool BoundariesMassVolumeCalculated;

private:
	void _compute_density_change();
	void _compute_density_adv(real dt);
	real _compute_density_error(const real offset);
	real _divergence_solver_iteration(real dt);
	real _pressure_solve_iteration(real dt);
	void _divergence_solver_iteration_kernel(real dt);
	void _pressure_solve_iteration_kernel(real dt);
};
}

#endif //HINAPE_DFSPH_AKINCI_H
