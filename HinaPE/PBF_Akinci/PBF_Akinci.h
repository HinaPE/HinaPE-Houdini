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
#include "common/geometry.h"
#include <vector>
#include <UT/UT_Vector3.h>
#include <UT/UT_Matrix4.h>
#include <UT/UT_Quaternion.h>
namespace HinaPE
{
using real = float;
using Vector = UT_Vector3T<real>;
using PolyKernel = Poly6<real, Vector>;
using Kernel = ICubic<real, Vector>;
using ScalarArrayCPU = std::vector<real>;
using VectorArrayCPU = std::vector<Vector>;
using FluidCPU = IFluid<real, Vector, ScalarArrayCPU, VectorArrayCPU>;
using AkinciBoundaryCPU = IAkinciBoundary<real, Vector, ScalarArrayCPU, VectorArrayCPU>;
using NeighborBuilder = NeighborBuilderGPU<real, Vector, ScalarArrayCPU, VectorArrayCPU>;
using FluidEmitter = IFluidEmitter<real, Vector, ScalarArrayCPU, VectorArrayCPU>;
using Surface = ISurface<real, Vector, Quaternion>;

struct SDFBoundaryPBF
{
    std::shared_ptr<Surface> S = nullptr; // use for accurate collision detection (RigidBody Solver should use Convex Mesh, but this (Cubby Flow Geometry) can use any accurate Geometry Mesh)
};

struct AkinciBoundaryPBF : public AkinciBoundaryCPU
{
    VectorArrayCPU x_init;
    UT_DMatrix4 xform; // update from outside. if Boundary is static, [xform] is the center of mass of the boundary, and keep the same all the time // if Boundary is dynamic, [xform] is updated by RigidBody simulator (update from outside)
    std::vector<int> boundary_sp; // Static Particles
    VectorArrayCPU normals;
    VectorArrayCPU u_diff;
};

struct PBF_AkinciFluid : public FluidCPU
{
    VectorArrayCPU pred_x;
    ScalarArrayCPU lambda;
    VectorArrayCPU delta_p;
    VectorArrayCPU a_ext;
};

struct PBF_AkinciParam
{
    real FLUID_REST_DENSITY = 1000.0f;
    real FLUID_PARTICLE_RADIUS = 0.01;
    real FLUID_SURFACE_TENSION = 0.01;
    real FLUID_KERNAL_RADIUS = 0.04;
    real FLUID_VISCOSITY = 0.01;
    real BOUNDARY_VISCOSITY = 0;
    Vector GRAVITY = Vector(0, -9.8, 0);
    bool TOP_OPEN = true;
    bool ENABLE_VISCOSITY = true;

    real EPSILON = 1e-6;
    int MAX_ITERATIONS = 5;

    std::vector<real> BOUNDARY_REST_DENSITY;
    std::vector<bool> BOUNDARY_DYNAMICS;
};

struct PBF_SDFParam
{
    std::vector<real> SDF_FRICTION;
    std::vector<real> SDF_BOUNCINESS;
    std::vector<bool> SDF_DYNAMICS;
};

struct PBF_AkinciSolver : public PBF_AkinciParam, public PBF_SDFParam
{
	PBF_AkinciSolver(real, Vector);
	virtual void Solve(real dt);
	std::shared_ptr<PBF_AkinciFluid> Fluid;
    std::vector<std::shared_ptr<AkinciBoundaryPBF>> Boundaries;
    std::vector<std::shared_ptr<SDFBoundaryPBF>> SDFBoundaries;

protected:
    void build_neighbors();
    void compute_density();
    void predict_position(real dt);
    void solve_density_constraints();
    void compute_lambda();
    void compute_delta_p();
    void update_predict_position();
    void update_position_and_velocity(real dt);
    void enforce_boundary();
    void compute_external_a();

private:
	void _for_each_fluid_particle(const std::function<void(size_t, Vector)> &);
	void _for_each_neighbor_fluid(size_t, const std::function<void(size_t, Vector)> &);
    void _for_each_fluid_particle_pred(const std::function<void(size_t, Vector)> &);
    void _for_each_neighbor_boundaries(size_t, const std::function<void(size_t, Vector, size_t)> &);
    void _update_akinci_boundaries();
    void _resize();
	NeighborBuilder NeighborBuilder;
	Vector MaxBound;
	bool NeighborBuilderInited;
    bool BoundariesMassVolumeCalculated;
};
}

#endif //HINAPE_PBF_AKINCI_H
