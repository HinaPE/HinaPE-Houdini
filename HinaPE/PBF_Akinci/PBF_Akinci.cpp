#include "PBF_Akinci.h"

#include <numeric>
#include <utility>
#include <execution>

HinaPE::PBF_AkinciSolver::PBF_AkinciSolver(HinaPE::real _r, HinaPE::Vector _b)
		: NeighborBuilder(_r), MaxBound(_b / 2.), NeighborBuilderInited(false)
{
    PolyKernel::set_radius(_r);
	Fluid = std::make_shared<PBF_AkinciFluid>();
}
void HinaPE::PBF_AkinciSolver::Solve(HinaPE::real dt)
{
    _resize();
    _update_akinci_boundaries();
    predict_position(dt);
    solve_density_constraints();
    update_position_and_velocity(dt);
    enforce_boundary();
}
void HinaPE::PBF_AkinciSolver::_resize()
{
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
        Fluid->lambda.resize(n);
        Fluid->delta_p.resize(n);
        Fluid->a_ext.resize(n);
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

void HinaPE::PBF_AkinciSolver::_update_akinci_boundaries()
{
    for (auto &Boundary: Boundaries)
    {
        //std::cout << "Boundary->xfom: " << Boundary->xform << std::endl;
        std::transform(Boundary->x_init.begin(), Boundary->x_init.end(), Boundary->x.begin(), [&](Vector x) { return rowVecMult(x, Boundary->xform); });
        std::fill(Boundary->v.begin(), Boundary->v.end(), Vector{0, 0, 0});
        std::fill(Boundary->a.begin(), Boundary->a.end(), Vector{0, 0, 0});
    }
}

void HinaPE::PBF_AkinciSolver::build_neighbors()
{
	if (!NeighborBuilderInited)
	{
		real d = static_cast<real>(2.f) * FLUID_PARTICLE_RADIUS;
		std::fill(Fluid->V.begin(), Fluid->V.end(), static_cast<real>(.8f) * d * d * d);
		std::transform(Fluid->V.begin(), Fluid->V.end(), Fluid->m.begin(), [&](real V) { return V * FLUID_REST_DENSITY; });
        std::transform(Fluid->x.begin(), Fluid->x.end(), Fluid->pred_x.begin(), [](Vector x) { return x; });
		std::vector<VectorArrayCPU *> x_sets;
		x_sets.emplace_back(&Fluid->pred_x);
        for (auto &Boundary: Boundaries)
            x_sets.emplace_back(&Boundary->x); //// TODO:!
		NeighborBuilder.init(x_sets);
		NeighborBuilderInited = true;
	} else
		//NeighborBuilder.resize_set(0, &Fluid->pred_x); // if new fluid particles is generated, update fluid set

	NeighborBuilder.update_set(0); // 0 means fluid set
    serial_for(Boundaries.size(), [&](size_t b_set)
    {
        if (BOUNDARY_DYNAMICS[b_set])
            NeighborBuilder.update_set(b_set + 1); // b_set + 1 means #b_set boundary set
    });
	NeighborBuilder.build();

    _for_each_fluid_particle_pred(
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

void HinaPE::PBF_AkinciSolver::compute_density()
{
    _for_each_fluid_particle_pred(
			[&](size_t i, Vector x_i)
			{
				real rho_i = Fluid->V[i] * PolyKernel::W_zero();
				_for_each_neighbor_fluid(i, [&](size_t j, Vector x_j)
				{
					rho_i += Fluid->V[j] * PolyKernel::W(x_i - Fluid->pred_x[j]);
				});
                _for_each_neighbor_boundaries(i, [&](size_t j, Vector x_j, size_t b_set)
                {
                    rho_i += Boundaries[b_set]->V[j] * PolyKernel::W(x_i - x_j);
                });
				Fluid->rho[i] = rho_i * FLUID_REST_DENSITY;
			});
}

void HinaPE::PBF_AkinciSolver::enforce_boundary()
{
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
void HinaPE::PBF_AkinciSolver::_for_each_fluid_particle(const std::function<void(size_t, Vector)> &f)
{
	parallel_for(Fluid->size, [&](size_t i) { f(i, Fluid->x[i]); });
}
void HinaPE::PBF_AkinciSolver::_for_each_neighbor_fluid(size_t i, const std::function<void(size_t, Vector)> &f)
{
	NeighborBuilder.for_each_neighbor(0, 0, i, f);
}
void HinaPE::PBF_AkinciSolver::_for_each_fluid_particle_pred(const std::function<void(size_t, Vector)> &f) {
    parallel_for(Fluid->size, [&](size_t i) { f(i, Fluid->pred_x[i]); });
}

void HinaPE::PBF_AkinciSolver::_for_each_neighbor_boundaries(size_t i, const std::function<void(size_t, Vector, size_t)> &f)
{
    serial_for(Boundaries.size(), [&](size_t b_set)
    {
        NeighborBuilder.for_each_neighbor(0, b_set + 1, i, [&](size_t j, Vector x_j) { f(j, x_j, b_set); });
    });
}

void HinaPE::PBF_AkinciSolver::predict_position(HinaPE::real dt) {
    compute_external_a();
    _for_each_fluid_particle([&](size_t i, Vector x_i)
                             {
                                 Fluid->v[i] += dt * Fluid->a_ext[i];
                                 Fluid->pred_x[i] = Fluid->x[i] + dt * Fluid->v[i];
                             });
}

void HinaPE::PBF_AkinciSolver::compute_external_a() {
    _for_each_fluid_particle([&](size_t i, Vector x_i)
                             {
                                 Fluid->a_ext[i] = GRAVITY;
                             });
}

void HinaPE::PBF_AkinciSolver::solve_density_constraints() {
    int iter = 0;
    while(iter < MAX_ITERATIONS)
    {
        build_neighbors();
        compute_density();
        compute_lambda();
        compute_delta_p();
        update_predict_position();
        iter++;
    }
}

void HinaPE::PBF_AkinciSolver::compute_lambda() {
    _for_each_fluid_particle_pred([&](size_t i, Vector pred_x_i)
      {
          real C_i = Fluid->rho[i] / FLUID_REST_DENSITY - 1;
          if(C_i > 0)
          {
              real sum_grad_C_i_squared = 0;
              Vector grad_C_i = Vector(0, 0, 0);
              _for_each_neighbor_fluid(i, [&](size_t j, Vector x_j)
              {
                  Vector grad_C_ij = - Fluid->V[i]  * PolyKernel::gradW(pred_x_i - Fluid->pred_x[j]);
                  sum_grad_C_i_squared += grad_C_ij.length2();
                  grad_C_i -= grad_C_ij;
              });
              _for_each_neighbor_boundaries(i, [&](size_t j, Vector x_j, size_t b_set)
              {
                  Vector grad_C_ij = - Boundaries[b_set]->V[j] * PolyKernel::gradW(pred_x_i - x_j);
                  sum_grad_C_i_squared += grad_C_ij.length2();
                  grad_C_i -= grad_C_ij;
              });
              sum_grad_C_i_squared += grad_C_i.length2();
              Fluid->lambda[i] = - C_i / (sum_grad_C_i_squared + EPSILON);
          }else{
              Fluid->lambda[i] = 0;
          }
      });
}

void HinaPE::PBF_AkinciSolver::compute_delta_p() {
    _for_each_fluid_particle_pred([&](size_t i, Vector pred_x_i)
      {
          auto k_corr = Fluid->m[i] * 1.0e-04;
          auto n_corr = 4.0;
          auto q_corr = 0.1;

          Vector delta_p_i = Vector(0, 0, 0);
          _for_each_neighbor_fluid(i, [&](size_t j, Vector x_j)
          {
              const auto w_corr = PolyKernel::W(q_corr * FLUID_PARTICLE_RADIUS);
              const auto ratio = PolyKernel::W((pred_x_i - Fluid->pred_x[j]).length()) / w_corr;
              /*if((pred_x_i - Fluid->pred_x[j]).length() == 0)
                  std::cout << "r is zero" << std::endl;
              if(PolyKernel::gradW(pred_x_i - Fluid->pred_x[j]) == Vector(0, 0, 0))
              std::cout << "gradW is zero" << std::endl;*/
              const auto s_corr = -k_corr * pow(ratio, n_corr);

              Vector grad_C_ij = - Fluid->V[i]  * PolyKernel::gradW(pred_x_i - Fluid->pred_x[j]);
              delta_p_i -= (Fluid->lambda[i] + Fluid->lambda[j] + s_corr) * grad_C_ij;
          });
            _for_each_neighbor_boundaries(i, [&](size_t j, Vector x_j, size_t b_set)
            {
                Vector grad_C_ij = - Fluid->V[i]  * PolyKernel::gradW(pred_x_i - x_j);
                delta_p_i += (Fluid->lambda[i]) * grad_C_ij;
            });
          Fluid->delta_p[i] = delta_p_i;
      });
}

void HinaPE::PBF_AkinciSolver::update_predict_position() {
    _for_each_fluid_particle_pred([&](size_t i, Vector pred_x_i)
      {
          Fluid->pred_x[i] -= Fluid->delta_p[i];
      });
}

void HinaPE::PBF_AkinciSolver::update_position_and_velocity(HinaPE::real dt) {
    _for_each_fluid_particle([&](size_t i, Vector x_i)
                             {
                                 Fluid->v[i] = (Fluid->pred_x[i] - Fluid->x[i]) / dt;
                             });

    if(ENABLE_VISCOSITY)
    {
        real c_v = FLUID_VISCOSITY;
        _for_each_fluid_particle([&](size_t i, Vector x_i)
         {
             Vector viscosity_force{0, 0, 0};
             _for_each_neighbor_fluid(i, [&](size_t j, Vector x_j)
             {
                 Vector r = Fluid->pred_x[i] - Fluid->pred_x[j];
                 real r_len = r.length();
                 if(r_len > 1e-6)
                 {
                     Vector u = Fluid->v[i] - Fluid->v[j];
                     u *= PolyKernel::W(r) * Fluid->V[j];
                     viscosity_force += u;
                 }
             });

             _for_each_neighbor_boundaries(i, [&](size_t j, Vector x_j, size_t b_set)
             {
                 Vector r = Fluid->pred_x[i] - x_j;
                 const real r2 = r.dot(r);
                 real v_xy = (Fluid->v[i] - Fluid->v[j]).dot(r);
                 const real kr2 = PolyKernel::_r * PolyKernel::_r;
                 Vector f_v = 10 * BOUNDARY_VISCOSITY * (BOUNDARY_REST_DENSITY[b_set] * Boundaries[b_set]->V[j] / Fluid->rho[i]) * v_xy / (r2 + 0.01f * kr2) * PolyKernel::gradW(r);
                 viscosity_force += f_v;
             });

             Fluid->v[i] = Fluid->v[i] - c_v * viscosity_force;
         });
    }

    _for_each_fluid_particle([&](size_t i, Vector x_i)
                             {
                                 Fluid->x[i] = Fluid->pred_x[i];
                             });
}



