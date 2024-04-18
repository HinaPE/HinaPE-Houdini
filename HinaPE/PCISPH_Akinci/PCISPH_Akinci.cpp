//
// Created by LiYifan on 2024/4/18.
//

#include "PCISPH_Akinci.h"

#include <numeric>
#include <utility>
#include <execution>

HinaPE::PCISPH_AkinciSolver::PCISPH_AkinciSolver(HinaPE::real _r, HinaPE::Vector _b): NeighborBuilder(_r), MaxBound(_b / 2.), NeighborBuilderInited(false)
{
    PolyKernel::set_radius(_r);
    SpikyKernel::set_radius(_r);
    Fluid = std::make_shared<PCISPH_AkinciFluid>();
    FLUID_KERNAL_RADIUS = _r;
}

void HinaPE::PCISPH_AkinciSolver::Solve(HinaPE::real dt) {
    _resize();
    _update_akinci_boundaries();
    real delta = _compute_delta(dt);
    build_neighbors();
    compute_density();
    accumulate_non_pressure_force();
    initialize_pressure_and_pressure_force();
    prediction_correction_step(dt,delta);
    correct_velocity_and_position(dt);
    enforce_boundary();
}

void HinaPE::PCISPH_AkinciSolver::build_neighbors() {
    if (!NeighborBuilderInited)
    {
        real d = static_cast<real>(2.f) * FLUID_PARTICLE_RADIUS;
        std::fill(Fluid->V.begin(), Fluid->V.end(), static_cast<real>(.8f) * d * d * d);
        std::transform(Fluid->V.begin(), Fluid->V.end(), Fluid->m.begin(), [&](real V) { return V * FLUID_REST_DENSITY; });

        std::vector<VectorArrayCPU *> x_sets;
        x_sets.emplace_back(&Fluid->x);
        for (auto &Boundary: Boundaries)
            x_sets.emplace_back(&Boundary->x);
        NeighborBuilder.init(x_sets);
        NeighborBuilderInited = true;
    } else
        NeighborBuilder.resize_set(0, &Fluid->x); // if new fluid particles is generated, update fluid set

    NeighborBuilder.update_set(0); // 0 means fluid set
    serial_for(Boundaries.size(), [&](size_t b_set)
    {
        if (BOUNDARY_DYNAMICS[b_set])
            NeighborBuilder.update_set(b_set + 1); // b_set + 1 means #b_set boundary set
    });
    NeighborBuilder.build();

    // This can be deleted for performance
    _for_each_fluid_particle(
            [&](size_t i, Vector x_i)
            {
                Fluid->neighbor_this[i] = NeighborBuilder.n_neighbors(0, 0, i);
                Fluid->neighbor_others[i] = 0;
                serial_for(Boundaries.size(), [&](size_t b_set)
                {
                    Fluid->neighbor_others[i] += NeighborBuilder.n_neighbors(0, b_set + 1, i);
                });
            });

    if (!BoundariesMassVolumeCalculated)
    {
        // compute V and m for Akinci Boundaries
        serial_for(Boundaries.size(), [&](size_t b_set)
        {
            parallel_for(Boundaries[b_set]->size, [&](size_t i)
            {
                real V = PolyKernel::W_zero();
                Vector x_i = Boundaries[b_set]->x[i];
                NeighborBuilder.for_each_neighbor(
                        b_set + 1, b_set + 1, i,
                        [&](size_t j, Vector x_j)
                        {
                            V += PolyKernel::W(x_i - x_j);
                        });
                Boundaries[b_set]->V[i] = static_cast<real>(1.f) / V;
                Boundaries[b_set]->m[i] = Boundaries[b_set]->V[i] * BOUNDARY_REST_DENSITY[b_set];
                Boundaries[b_set]->neighbor_this[i] = NeighborBuilder.n_neighbors(b_set + 1, b_set + 1, i);
            });
            serial_for(Boundaries.size(), [&](size_t b_set)
            {
                if (!BOUNDARY_DYNAMICS[b_set])
                    NeighborBuilder.disable_set_to_search_from(b_set + 1); // for performance, we don't need to search from static boundary set (BUT STILL NEED TO BE SEARCHED FROM)
            });
        });
        BoundariesMassVolumeCalculated = true;
    }
}

void HinaPE::PCISPH_AkinciSolver::compute_density() {
    _for_each_fluid_particle([&](size_t i, Vector x_i)
    {
        real rho_i = Fluid->V[i] * PolyKernel::W_zero();
        _for_each_neighbor_fluid(i, [&](size_t j, Vector x_j)
        {
            rho_i += Fluid->V[j] * PolyKernel::W(x_i - x_j);
        });
        _for_each_neighbor_boundaries(i, [&](size_t j, Vector x_j, size_t b_set)
        {
            rho_i += Boundaries[b_set]->V[j] * PolyKernel::W(x_i - x_j);
        });
        Fluid->rho[i] = rho_i * FLUID_REST_DENSITY;
    });
}

void HinaPE::PCISPH_AkinciSolver::accumulate_non_pressure_force() {
_for_each_fluid_particle([&](size_t i, Vector x_i)
    {
        Fluid->a_ext[i] = GRAVITY;
    });
}

void HinaPE::PCISPH_AkinciSolver::initialize_pressure_and_pressure_force() const {
    std::fill(Fluid->pressure.begin(), Fluid->pressure.end(), 0);
    std::fill(Fluid->a_pressure.begin(), Fluid->a_pressure.end(), Vector(0, 0, 0));
    std::fill(Fluid->d_error.begin(), Fluid->d_error.end(), 0);
}

void HinaPE::PCISPH_AkinciSolver::prediction_correction_step(real dt, real delta)
{
    int iteration = 0;
    while(((iteration < MIN_ITERATIONS)||(DENSITY_ERROR_TOO_LARGE))&&(iteration < MAX_ITERATIONS))
    {
        // algorithm line 9~11
        predict_velocity_and_position(dt);
        // algorithm line 12~13
        predict_density();
        // algorithm line 14~15
        update_pressure(delta);
        // algorithm line 16~17
        accumulate_pressure_force();
        iteration++;
    }
}

void HinaPE::PCISPH_AkinciSolver::predict_velocity_and_position(real dt){
    _for_each_fluid_particle([&](size_t i, Vector x_i)
    {
        Fluid->pred_v[i] = Fluid->v[i] + dt * (Fluid->a_ext[i] + Fluid->a_pressure[i]);
        Fluid->pred_x[i] = Fluid->x[i] + dt * Fluid->pred_v[i];
    });
}

void HinaPE::PCISPH_AkinciSolver::predict_density()
{
    _for_each_fluid_particle([&](size_t i, Vector x_i)
    {
        real rho_i = Fluid->V[i] * PolyKernel::W_zero();
        _for_each_neighbor_fluid(i, [&](size_t j, Vector x_j)
        {
            rho_i += Fluid->V[j] * PolyKernel::W(Fluid->pred_x[i] - Fluid->pred_x[j]);
        });
        _for_each_neighbor_boundaries(i, [&](size_t j, Vector x_j, size_t b_set)
        {
            rho_i += Boundaries[b_set]->V[j] * PolyKernel::W(Fluid->pred_x[i] - x_j);
        });
        Fluid->pred_density[i] = rho_i * FLUID_REST_DENSITY;
    });
}

void HinaPE::PCISPH_AkinciSolver::update_pressure(real delta) {
    _for_each_fluid_particle([&](size_t i, Vector x_i)
    {
        real density_error = Fluid->pred_density[i] - FLUID_REST_DENSITY;
        real pressure = delta * Fluid->d_error[i];
        if(pressure < 0)
        {
            pressure *= 0;
            density_error *= 0;
        }
        Fluid->d_error[i] = density_error;
        Fluid->pressure[i] += pressure;
    });

}

void HinaPE::PCISPH_AkinciSolver::accumulate_pressure_force()
{
    _for_each_fluid_particle([&](size_t i, Vector x_i)
    {
        Vector f_pressure(0, 0, 0);
        _for_each_neighbor_fluid(i, [&](size_t j, Vector x_j)
        {
            real dist = (Fluid->pred_x[i] - Fluid->pred_x[j]).length();
            if(dist > EPSILON && Fluid->pred_density[j] > EPSILON)
                f_pressure -= Fluid->V[j] * (Fluid->pressure[i] / (Fluid->pred_density[i] * Fluid->pred_density[i]) + Fluid->pressure[j] / (Fluid->pred_density[j] * Fluid->pred_density[j])) * SpikyKernel::gradW(Fluid->pred_x[j] - Fluid->pred_x[i]);
        });
        _for_each_neighbor_boundaries(i, [&](size_t j, Vector x_j, size_t b_set)
        {
            real dist = (Fluid->pred_x[i] - x_j).length();
            Vector Boundary_force = -(Fluid->pressure[i] / (Fluid->pred_density[i] * Fluid->pred_density[i])) * SpikyKernel::gradW(x_j - Fluid->pred_x[i]);
            f_pressure += Boundaries[b_set]->V[j] * Boundary_force;

            if (BOUNDARY_DYNAMICS[b_set]) // Dynamic Rigid Body
                Boundaries[b_set]->a[j] += -Boundary_force;
        });
        Fluid->a_pressure[i] = f_pressure;
    });
}

void HinaPE::PCISPH_AkinciSolver::correct_velocity_and_position(real dt) {
    _for_each_fluid_particle([&](size_t i, Vector x_i)
    {
        Fluid->v[i] += (Fluid->a_pressure[i] + Fluid->a_ext[i]) * dt;
        Fluid->x[i] += Fluid->v[i] * dt;
    });

}

HinaPE::real HinaPE::PCISPH_AkinciSolver::_compute_delta(real dt) {
    UT_Vector3 sumGradW = UT_Vector3(0, 0, 0);
    fpreal sumGradW2 = 0.0;
    const float supportRadius = FLUID_KERNAL_RADIUS;
    const fpreal diam = static_cast<fpreal>(2.0) * FLUID_PARTICLE_RADIUS;
    const UT_Vector3 xi(0, 0, 0);
    UT_Vector3 xj = {-supportRadius, -supportRadius, -supportRadius};

    while(xj.x() <= supportRadius)
    {
        while (xj.y() <= supportRadius)
        {
            while (xj.z() <= supportRadius)
            {
                fpreal dist = (xi - xj).length();
                if (dist * dist < supportRadius * supportRadius)
                {
                    UT_Vector3 dir = xi - xj;
                    const UT_Vector3 gradW = SpikyKernel::gradW(dir);
                    sumGradW += gradW;
                    sumGradW2 += gradW.dot(gradW);
                }
                xj.z() += diam;
            }
            xj.y() += diam;
            xj.z() = -supportRadius;
        }
        xj.x() += diam;
        xj.y() = -supportRadius;
        xj.z() = -supportRadius;
    }

    fpreal denom = -sumGradW.dot(sumGradW) - sumGradW2;
    fpreal beta = 2 * dt * dt * Fluid->V[0] * Fluid->V[0];
    return (std::fabs(denom) > 0) ? -1 / (beta * denom) : 0;
}

void HinaPE::PCISPH_AkinciSolver::enforce_boundary() {
    _for_each_fluid_particle(
            [&](size_t i, Vector x_i)
            {
                Vector collision_normal{0, 0, 0};
                if (x_i.x() > MaxBound.x())
                {
                    Fluid->x[i].x() = MaxBound.x();
                    collision_normal.x() += 1;
                }
                if (x_i.x() < -MaxBound.x())
                {
                    Fluid->x[i].x() = -MaxBound.x();
                    collision_normal.x() -= 1;
                }
                if (!TOP_OPEN)
                {
                    if (x_i.y() > MaxBound.y())
                    {
                        Fluid->x[i].y() = MaxBound.y();
                        collision_normal.y() += 1;
                    }
                }
                if (x_i.y() < -MaxBound.y())
                {
                    Fluid->x[i].y() = -MaxBound.y();
                    collision_normal.y() -= 1;
                }
                if (x_i.z() > MaxBound.z())
                {
                    Fluid->x[i].z() = MaxBound.z();
                    collision_normal.z() += 1;
                }
                if (x_i.z() < -MaxBound.z())
                {
                    Fluid->x[i].z() = -MaxBound.z();
                    collision_normal.z() -= 1;
                }
                collision_normal.normalize();
                constexpr real c_f = static_cast<real>(0.5f);
                Fluid->v[i] -= (1. + c_f) * Fluid->v[i].dot(collision_normal) * collision_normal;
            });
}

void HinaPE::PCISPH_AkinciSolver::_for_each_fluid_particle(const std::function<void(size_t i, Vector x_i)> &f) {
    parallel_for(Fluid->size, [&](size_t i) { f(i, Fluid->x[i]); });
}

void HinaPE::PCISPH_AkinciSolver::_for_each_neighbor_fluid(size_t i, const std::function<void(size_t, Vector)> &f) {
    NeighborBuilder.for_each_neighbor(0, 0, i, f);
}

void HinaPE::PCISPH_AkinciSolver::_for_each_neighbor_boundaries(size_t i, const std::function<void(size_t, Vector, size_t)> &f) {
    serial_for(Boundaries.size(), [&](size_t b_set)
    {
        NeighborBuilder.for_each_neighbor(0, b_set + 1, i, [&](size_t j, Vector x_j) { f(j, x_j, b_set); });
    });
}

void HinaPE::PCISPH_AkinciSolver::_resize() {
/**
	 * For Fluid, [Fluid->x] is inited or update from outside, but [Fluid->size] is never updated from outside
	 * If [Fluid->size] is not equal to [Fluid->x.size()], then resize all the vectors
	 */
    size_t pre_size = Fluid->size;
    //std::cout << "Fluid->size: " << Fluid->size << std::endl;
    if (Fluid->size != Fluid->x.size())
    {
        Fluid->size = Fluid->x.size();
        size_t n = Fluid->size;
        Fluid->x.resize(n);
        Fluid->v.resize(n);
        Fluid->a.resize(n);
        Fluid->m.resize(n);
        Fluid->V.resize(n);
        Fluid->rho.resize(n);
        Fluid->neighbor_this.resize(n);
        Fluid->neighbor_others.resize(Fluid->size);
        Fluid->a.resize(n);

        Fluid->pred_x.resize(n);
        Fluid->pred_v.resize(n);
        Fluid->pred_density.resize(n);
        Fluid->pressure.resize(n);
        Fluid->d_error.resize(n);
        Fluid->a_ext.resize(n);
        Fluid->a_pressure.resize(n);
    }

    /**
	 * For Boundaries, [Boundary->x_init] is inited from outside, but [Boundary->size] is never updated from outside
	 * If [Boundary->size] is not equal to [Boundary->x_init.size()], then resize all the vectors
	 */
    for (auto &Boundary: Boundaries)
    {
        //std::cout << "Boundary->size: " << Boundary->size << std::endl;
        if (Boundary->size != Boundary->x_init.size())
        {
            Boundary->size = Boundary->x_init.size();
            Boundary->x.resize(Boundary->size);
            Boundary->v.resize(Boundary->size);
            Boundary->a.resize(Boundary->size);
            Boundary->m.resize(Boundary->size);
            Boundary->V.resize(Boundary->size);
            Boundary->rho.resize(Boundary->size);
            Boundary->neighbor_this.resize(Boundary->size);
            Boundary->neighbor_others.resize(Boundary->size);

            Boundary->boundary_sp.resize(Boundary->size);
            Boundary->normals.resize(Boundary->size);
            Boundary->u_diff.resize(Boundary->size);
        }
    }
}

void HinaPE::PCISPH_AkinciSolver::_update_akinci_boundaries() {
    for (auto &Boundary: Boundaries)
    {
        //std::cout << "Boundary->xfom: " << Boundary->xform << std::endl;
        std::transform(Boundary->x_init.begin(), Boundary->x_init.end(), Boundary->x.begin(), [&](Vector x) { return rowVecMult(x, Boundary->xform); });
        std::fill(Boundary->v.begin(), Boundary->v.end(), Vector{0, 0, 0});
        std::fill(Boundary->a.begin(), Boundary->a.end(), Vector{0, 0, 0});
    }
}









