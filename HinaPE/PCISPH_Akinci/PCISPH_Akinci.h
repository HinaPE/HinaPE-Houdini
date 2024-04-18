//
// Created by LiYifan on 2024/4/18.
//

#ifndef HINAPE_HOUDINI_PCISPH_AKINCI_H
#define HINAPE_HOUDINI_PCISPH_AKINCI_H

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
    using SpikyKernel = Spiky<real, Vector>;
    using ScalarArrayCPU = std::vector<real>;
    using VectorArrayCPU = std::vector<Vector>;
    using FluidCPU = IFluid<real, Vector, ScalarArrayCPU, VectorArrayCPU>;
    using AkinciBoundaryCPU = IAkinciBoundary<real, Vector, ScalarArrayCPU, VectorArrayCPU>;
    using NeighborBuilder = NeighborBuilderGPU<real, Vector, ScalarArrayCPU, VectorArrayCPU>;
    using FluidEmitter = IFluidEmitter<real, Vector, ScalarArrayCPU, VectorArrayCPU>;
    using Surface = ISurface<real, Vector, Quaternion>;

    struct AkinciBoundaryPCISPH : public AkinciBoundaryCPU
    {
        VectorArrayCPU x_init;
        UT_DMatrix4 xform; // update from outside. if Boundary is static, [xform] is the center of mass of the boundary, and keep the same all the time // if Boundary is dynamic, [xform] is updated by RigidBody simulator (update from outside)
        std::vector<int> boundary_sp; // Static Particles
        VectorArrayCPU normals;
        VectorArrayCPU u_diff;
    };

    struct PCISPH_AkinciFluid : public FluidCPU
    {
        VectorArrayCPU pred_x;
        VectorArrayCPU pred_v;
        ScalarArrayCPU pred_density;
        ScalarArrayCPU pressure;
        ScalarArrayCPU d_error;
        VectorArrayCPU a_ext;
        VectorArrayCPU a_pressure;
        ScalarArrayCPU delta;
    };

    struct PCISPH_AkinciParam
    {
        real FLUID_PARTICLE_RADIUS = 0.01;
        real FLUID_REST_DENSITY = 1000.0f;
        real FLUID_KERNAL_RADIUS = 0.04;
        real FLUID_VISCOSITY = 0.1;
        real BOUNDARY_VISCOSITY = 0;
        real EPSILON = 1e-6;

        Vector GRAVITY = Vector(0, -9.8, 0);
        bool TOP_OPEN = true;
        bool ENABLE_VISCOSITY = true;
        real FLUID_KERNEL_RADIUS = 0.04;

        int MIN_ITERATIONS = 3;
        int MAX_ITERATIONS = 5;

        bool DENSITY_ERROR_TOO_LARGE = true;
        real MAX_DENSITY_ERROR_RATIO = 0.01;

        std::vector<real> BOUNDARY_REST_DENSITY;
        std::vector<bool> BOUNDARY_DYNAMICS;
    };

    struct PCISPH_AkinciSolver : public PCISPH_AkinciParam
    {
    public:
        PCISPH_AkinciSolver(real, Vector);
        void Solve(real dt);
        std::shared_ptr<PCISPH_AkinciFluid> Fluid;
        std::vector<std::shared_ptr<AkinciBoundaryPCISPH>> Boundaries;

    protected:
        void build_neighbors();
        void compute_density();
        void accumulate_non_pressure_force();
        void initialize_pressure_and_pressure_force() const;
        void prediction_correction_step(real dt);
        void predict_velocity_and_position(real dt);
        void predict_density();
        void update_pressure(real dt);
        void accumulate_pressure_force();
        void correct_velocity_and_position(real dt);
        void enforce_boundary();
    private:
        void _resize();
        void _update_akinci_boundaries();
        void _compute_delta(real dt);
        void _for_each_fluid_particle(const std::function<void(size_t, Vector)> &);
        void _for_each_neighbor_fluid(size_t, const std::function<void(size_t, Vector)> &);
        void _for_each_neighbor_boundaries(size_t i, const std::function<void(size_t, Vector, size_t)> &f);
        NeighborBuilder NeighborBuilder;
        Vector MaxBound;
        bool NeighborBuilderInited;
        bool BoundariesMassVolumeCalculated;
    };
}

#endif //HINAPE_HOUDINI_PCISPH_AKINCI_H
