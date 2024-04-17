//
// Created by LiYifan on 2024/3/4.
//

#ifndef HINAPE_PBF_H
#define HINAPE_PBF_H

#include "common/particles.h"
#include "common/emitter.h"
#include "common/kernels.h"
#include "common/neighbors.h"
#include "common/geometry.h"
#include <vector>
#include <UT/UT_Vector3.h>
namespace HinaPE {
    using real = float;
    using Vector = UT_Vector3T<real>;
    using PolyKernel = Poly6<real, Vector>;
    using ScalarArrayCPU = std::vector<real>;
    using VectorArrayCPU = std::vector<Vector>;
    using FluidCPU = IFluid<real, Vector, ScalarArrayCPU, VectorArrayCPU>;
    using NeighborBuilder = NeighborBuilderGPU<real, Vector, ScalarArrayCPU, VectorArrayCPU>;
    using FluidEmitter = IFluidEmitter<real, Vector, ScalarArrayCPU, VectorArrayCPU>;

    struct PbfFluidCPU : public FluidCPU {
        VectorArrayCPU pred_x;
        ScalarArrayCPU lambda;
        VectorArrayCPU delta_p;
        VectorArrayCPU a_ext;
    };

    struct PbfParamCPU {
        real FLUID_PARTICLE_RADIUS = 0.01;
        real FLUID_REST_DENSITY = 1000.0f;

        real FLUID_VISCOSITY = 0.1;
        real FLUID_SURFACE_TENSION = 0.01;
        real FLUID_VORTICITY = 0.01;
        real BOUNDARY_VISCOSITY = 0;

        real EPSILON = 1e-6;
        int MAX_ITERATIONS = 5;

        Vector GRAVITY = Vector(0, -9.8, 0);
        bool TOP_OPEN = true;

        bool ENABLE_VISCOSITY = true;
        bool ENABLE_SURFACE_TENSION = true;
        bool ENABLE_VORTICITY = true;

        real FLUID_KERNEL_RADIUS = 0.04;
    };

    struct PbfSolver : public PbfParamCPU {
    public:
        PbfSolver(real, Vector);

        void Solve(real dt);

    public:
        std::shared_ptr<PbfFluidCPU> Fluid;

        NeighborBuilder NeighborBuilder;
        Vector MaxBound;
    public:
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

    public:
        void _for_each_fluid_particle(const std::function<void(size_t, Vector)> &);

        void _for_each_neighbor_fluid(size_t, const std::function<void(size_t, Vector)> &);

        void _resize();

        void _for_each_fluid_particle_pred(const std::function<void(size_t, Vector)> &);
    };
}



#endif //HINAPE_HOUDINI_PBF_H
