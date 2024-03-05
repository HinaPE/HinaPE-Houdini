#ifndef HINAPE_PBF_AKINCI_H
#define HINAPE_PBF_AKINCI_H

/**
 * PBF implementation, Akinci Boundary
 * paper: https://mmacklin.com/pbf_sig_preprint.pdf
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

struct PBF_AkinciFluid : public FluidCPU
{
	VectorArrayCPU p_x;
};

struct PBF_AkinciParam
{
	real FLUID_REST_DENSITY = 1000.0f;
	real FLUID_PARTICLE_RADIUS = 0.01;
	real FLUID_SURFACE_TENSION = 0.01;
	real FLUID_VISCOSITY = 0.01;
	Vector GRAVITY = Vector(0, -9.8, 0);
	bool TOP_OPEN = true;

	size_t PBF_ITERATION = 5;
	real MAX_DENSITY_ERROR = 0.05f;
};

struct PBF_AkinciSolver : public PBF_AkinciParam
{
	PBF_AkinciSolver(real, Vector);
	virtual void Solve(real dt);
	std::shared_ptr<PBF_AkinciFluid> Fluid;

protected:
	void resize();
	void predict_x(real dt);
	void build_neighbors();
	void pbf_iteration(real dt);
	void update_v(real dt);
	void compute_density();
	void non_pressure_force();
	void update_x(real dt);
	void enforce_boundary();

private:
	void _for_each_fluid_particle(const std::function<void(size_t, Vector)> &);
	void _for_each_neighbor_fluid(size_t, const std::function<void(size_t, Vector)> &);
	NeighborBuilder NeighborBuilder;
	Vector MaxBound;
	bool NeighborBuilderInited;

private:
	real _pressure_solver_iteration(real dt);
};
}

#endif //HINAPE_PBF_AKINCI_H
