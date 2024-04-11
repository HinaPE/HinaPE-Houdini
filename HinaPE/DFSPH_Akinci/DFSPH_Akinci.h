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
#include "common/geometry.h"
#include <vector>
#include <UT/UT_Vector3.h>
#include <UT/UT_Matrix4.h>
#include <UT/UT_Quaternion.h>
namespace HinaPE
{
using real = float;
using Vector = UT_Vector3T<real>;
using Quaternion = UT_QuaternionT<real>;
using Kernel = ICubic<real, Vector>;
using ScalarArrayCPU = std::vector<real>;
using VectorArrayCPU = std::vector<Vector>;
using FluidCPU = IFluid<real, Vector, ScalarArrayCPU, VectorArrayCPU>;
using AkinciBoundaryCPU = IAkinciBoundary<real, Vector, ScalarArrayCPU, VectorArrayCPU>;
using NeighborBuilder = NeighborBuilderGPU<real, Vector, ScalarArrayCPU, VectorArrayCPU>;
using FluidEmitter = IFluidEmitter<real, Vector, ScalarArrayCPU, VectorArrayCPU>;
using Surface = ISurface<real, Vector, Quaternion>;

struct SDFBoundary
{
	std::shared_ptr<Surface> S = nullptr; // use for accurate collision detection (RigidBody Solver should use Convex Mesh, but this (Cubby Flow Geometry) can use any accurate Geometry Mesh)
};

struct AkinciBoundary : public AkinciBoundaryCPU
{
	VectorArrayCPU x_init;
	UT_DMatrix4 xform; // update from outside. if Boundary is static, [xform] is the center of mass of the boundary, and keep the same all the time // if Boundary is dynamic, [xform] is updated by RigidBody simulator (update from outside)
    std::vector<int> boundary_sp; // Static Particles
    VectorArrayCPU normals;
    VectorArrayCPU u_diff;
};

struct DFSPH_AkinciFluid : public FluidCPU
{
	ScalarArrayCPU factor;
	ScalarArrayCPU k;
	ScalarArrayCPU density_adv;

    std::vector<int> fluid_bflp;
    std::vector<int> fluid_vp;
    VectorArrayCPU omega;
    VectorArrayCPU predict_omega;
    VectorArrayCPU omega_delta;
    VectorArrayCPU psi;
    VectorArrayCPU first_term, second_term;
    VectorArrayCPU refinement_omega;
};

struct DFSPH_AkinciParam
{
	real FLUID_REST_DENSITY = 1000.0f;
	real FLUID_PARTICLE_RADIUS = 0.01;
	real FLUID_SURFACE_TENSION = 0.01;
    real FLUID_KERNAL_RADIUS = 0.04;
	real FLUID_VISCOSITY = 0.01;
	real BOUNDARY_VISCOSITY = 0;
    real ALPHA = 0.1;
    real BETA = 0.2;
	Vector GRAVITY = /*Vector(0, -9.8, 0)*/Vector(5, 0, 0);
	bool TOP_OPEN = true;

	std::vector<real> BOUNDARY_REST_DENSITY;
	std::vector<bool> BOUNDARY_DYNAMICS;
};

struct DFSPH_SDFParam
{
	std::vector<real> SDF_FRICTION;
	std::vector<real> SDF_BOUNCINESS;
	std::vector<bool> SDF_DYNAMICS;
};

struct DFSPH_AkinciSolver : public DFSPH_AkinciParam, public DFSPH_SDFParam
{
	DFSPH_AkinciSolver(real, Vector);
	virtual void Solve(real dt);
	std::shared_ptr<DFSPH_AkinciFluid> Fluid;
	std::vector<std::shared_ptr<AkinciBoundary>> Boundaries;
	std::vector<std::shared_ptr<SDFBoundary>> SDFBoundaries;
public:
    void findBFLPs();
    void findSPs();
    void findVPs();
    void compute_vorticity_n_sph(real dt);
    void compute_vorticity_n1_sph();
    void compute_ideal_vorticity_n1_vorticity_equation(real dt);
    void compute_vorticity_dissipation();
    void compute_stream_function();
    void compute_vorticity_velocity();
    void clearMarkedParticles();
    void MarkVPs();
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
	void enforce_SDF_boundary();
	void enforce_boundary();

private:
	void _for_each_fluid_particle(const std::function<void(size_t, Vector)> &);
	void _for_each_neighbor_fluid(size_t, const std::function<void(size_t, Vector)> &);
	void _for_each_neighbor_boundaries(size_t, const std::function<void(size_t, Vector, size_t)> &);
    //void _for_each_boundary_particle(const std::function<void(size_t, Vector)> &);
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
