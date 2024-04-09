#include "DFSPH_Akinci.h"

#include <numeric>
#include <utility>
#include <execution>

HinaPE::DFSPH_AkinciSolver::DFSPH_AkinciSolver(HinaPE::real _r, HinaPE::Vector _b)
		: NeighborBuilder(_r), MaxBound(_b / 2.), NeighborBuilderInited(false), BoundariesMassVolumeCalculated(false)
{
	Kernel::set_radius(_r);
	Fluid = std::make_shared<DFSPH_AkinciFluid>();
    FLUID_KERNAL_RADIUS = _r;
}
void HinaPE::DFSPH_AkinciSolver::Solve(HinaPE::real dt)
{
	resize();
	update_akinci_boundaries();
    clearMarkedParticles();
	build_neighbors();
    compute_density();

    findBFLPs();
    findSPs();

    compute_vorticity_n_sph();

    ////////DFSPH/////////
	compute_factor();
	divergence_solve(dt);
	non_pressure_force();
	predict_velocity(dt);
    pressure_solve(dt);
    /////////////////////

    findVPs();
    /*MarkVPs();*/

    compute_vorticity_n1_sph();
    compute_ideal_vorticity_n1_vorticity_equation(dt);
    compute_vorticity_dissipation();
    compute_stream_function();
    compute_vorticity_velocity();

	advect(dt);

	//enforce_SDF_boundary();
	//enforce_boundary();
}
void HinaPE::DFSPH_AkinciSolver::resize()
{
	/**
	 * For Fluid, [Fluid->x] is inited or update from outside, but [Fluid->size] is never updated from outside
	 * If [Fluid->size] is not equal to [Fluid->x.size()], then resize all the vectors
	 */
	size_t pre_size = Fluid->size;
	if (Fluid->size != Fluid->x.size())
	{
		Fluid->size = Fluid->x.size();
		Fluid->v.resize(Fluid->size);
		Fluid->a.resize(Fluid->size);
		Fluid->m.resize(Fluid->size);
		Fluid->V.resize(Fluid->size);
		Fluid->rho.resize(Fluid->size);
		Fluid->neighbor_this.resize(Fluid->size);
		Fluid->neighbor_others.resize(Fluid->size);

		Fluid->factor.resize(Fluid->size);
		Fluid->k.resize(Fluid->size);
		Fluid->density_adv.resize(Fluid->size);

        Fluid->fluid_bflp.resize(Fluid->size, false);
        Fluid->fluid_vp.resize(Fluid->size, false);
        Fluid->omega.resize(Fluid->size);
        Fluid->predict_omega.resize(Fluid->size);
        Fluid->omega_delta.resize(Fluid->size);
        Fluid->psi.resize(Fluid->size);
        Fluid->first_term.resize(Fluid->size);
        Fluid->second_term.resize(Fluid->size);
	}

	/**
	 * For Boundaries, [Boundary->x_init] is inited from outside, but [Boundary->size] is never updated from outside
	 * If [Boundary->size] is not equal to [Boundary->x_init.size()], then resize all the vectors
	 */
	for (auto &Boundary: Boundaries)
	{
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
void HinaPE::DFSPH_AkinciSolver::update_akinci_boundaries()
{
	for (auto &Boundary: Boundaries)
	{
        //std::cout << "Boundary->xfom: " << Boundary->xform << std::endl;
		std::transform(Boundary->x_init.begin(), Boundary->x_init.end(), Boundary->x.begin(), [&](Vector x) { return rowVecMult(x, Boundary->xform); });
		std::fill(Boundary->v.begin(), Boundary->v.end(), Vector{0, 0, 0});
		std::fill(Boundary->a.begin(), Boundary->a.end(), Vector{0, 0, 0});
        std::fill(Boundary->boundary_sp.begin(), Boundary->boundary_sp.end(), false);
        std::fill(Boundary->normals.begin(), Boundary->normals.end(), Vector{0, 0, 0});
        std::fill(Boundary->u_diff.begin(), Boundary->u_diff.end(), Vector{0, 0, 0});
	}
}
void HinaPE::DFSPH_AkinciSolver::build_neighbors()
{
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
				real V = Kernel::W_zero();
				Vector x_i = Boundaries[b_set]->x[i];
				NeighborBuilder.for_each_neighbor(
						b_set + 1, b_set + 1, i,
						[&](size_t j, Vector x_j)
						{
							V += Kernel::W(x_i - x_j);
						});
				Boundaries[b_set]->V[i] = static_cast<real>(1.f) / V;
				Boundaries[b_set]->m[i] = Boundaries[b_set]->V[i] * BOUNDARY_REST_DENSITY[b_set];
				Boundaries[b_set]->neighbor_this[i] = NeighborBuilder.n_neighbors(b_set + 1, b_set + 1, i);
			});
			/*serial_for(Boundaries.size(), [&](size_t b_set)
			{
				if (!BOUNDARY_DYNAMICS[b_set])
					NeighborBuilder.disable_set_to_search_from(b_set + 1); // for performance, we don't need to search from static boundary set (BUT STILL NEED TO BE SEARCHED FROM)
			});*/

		});
		BoundariesMassVolumeCalculated = true;
	}
}
void HinaPE::DFSPH_AkinciSolver::findBFLPs() {
    _for_each_fluid_particle([&](size_t i, Vector x_i)
     {
         Vector fluid_pos = x_i;
         _for_each_neighbor_boundaries(i, [&](size_t j, Vector x_j, size_t b_set)
            {
                if(BOUNDARY_DYNAMICS[b_set])
                {
                    Vector boundary_pos = x_j;
                    fpreal dis = (fluid_pos - boundary_pos).length();
                    if(dis < FLUID_KERNAL_RADIUS)
                        Fluid->fluid_bflp[i] = 1;
                }
            });
     });
}
void HinaPE::DFSPH_AkinciSolver::findSPs() {
    serial_for(Boundaries.size(), [&](size_t b_set)
    {
        if(BOUNDARY_DYNAMICS[b_set])
        {
            parallel_for(Boundaries[b_set]->size, [&](size_t i)
            {
                // compute boundary normals
                Vector n{0, 0, 0};
                n = Kernel::gradW(Vector{0, 0, 0}) * Boundaries[b_set]->V[i];
                NeighborBuilder.for_each_neighbor(b_set + 1, b_set + 1, i, [&](size_t j, Vector x_j)
                {
                    n += Kernel::gradW(x_j - Boundaries[b_set]->x[i]) * Boundaries[b_set]->V[j];
                });
                if(n.length() != 0)
                    Boundaries[b_set]->normals[i] = n / n.length();

                // compute the smoothing velocity difference
                Vector u_diff{0, 0, 0};
                fpreal W_sum = Kernel::W_zero();
                NeighborBuilder.for_each_neighbor(b_set + 1, 0, i, [&](size_t j, Vector x_j)
                {
                    Vector u_i = Boundaries[b_set]->v[i];
                    Vector u_j = Fluid->v[j];
                    Vector r = Boundaries[b_set]->x[i] - Fluid->x[j];
                    fpreal W = Kernel::W(r);
                    u_diff += (u_i - u_j)  * W;
                    W_sum += W;
                });
                if(W_sum != 0)
                    Boundaries[b_set]->u_diff[i] = u_diff / W_sum;
            });

            parallel_for(Boundaries[b_set]->size, [&](size_t i)
            {
                Vector n = Boundaries[b_set]->normals[i];
                NeighborBuilder.for_each_neighbor(b_set + 1, b_set + 1, i, [&](size_t j, Vector x_j)
                {
                    n += Boundaries[b_set]->normals[j];
                });
                if(n.length() != 0)
                    Boundaries[b_set]->normals[i] = n / n.length();
            });
        }
    });

    _for_each_fluid_particle([&](size_t i, Vector x_i)
     {
         if(Fluid->fluid_bflp[i] == 1)
         {
             _for_each_neighbor_boundaries(i, [&](size_t j, Vector x_j, size_t b_set)
             {
                 if(BOUNDARY_DYNAMICS[b_set]){
                     if(Boundaries[b_set]->u_diff[j].dot(Boundaries[b_set]->normals[j]) < 0)
                     {
                         Boundaries[b_set]->boundary_sp[j] = 1;
                     }
                 }
             });
         }
     });
}
void HinaPE::DFSPH_AkinciSolver::findVPs() {
    double alpha = ALPHA;
    serial_for(Boundaries.size(), [&](size_t b_set)
    {
        if(BOUNDARY_DYNAMICS[b_set])
        {
            parallel_for(Boundaries[b_set]->size, [&](size_t i)
            {
                if(Boundaries[b_set]->boundary_sp[i] == 1)
                {
                    NeighborBuilder.for_each_neighbor(b_set + 1, 0, i, [&](size_t j, Vector x_j)
                    {
                        if(Fluid->fluid_bflp[j] == 1)
                        {
                            //Fluid->fluid_bflp[j] = 2; // pink but not vp
                            Fluid->fluid_vp[j] = 1;
                            Vector v_ji = Fluid->v[j] - Boundaries[b_set]->v[i];
                            Vector x_ji = Fluid->x[j] - Boundaries[b_set]->x[i];
                            if(x_ji.dot(v_ji) > 0)
                            {
                                Vector v_tan_ji = v_ji - Boundaries[b_set]->normals[i] * v_ji.dot(Boundaries[b_set]->normals[i]);
                                float rand_num = 0.0 + (float)(rand()) / ((float)(RAND_MAX / (1.0 - 0.0)));
                                Fluid->omega[j] = (1.0 - alpha) * rand_num * v_tan_ji / v_tan_ji.length() + alpha * Fluid->omega[j];
                            }
                        }
                    });
                }
            });
        }
    });
}
void HinaPE::DFSPH_AkinciSolver::MarkVPs() {
    _for_each_fluid_particle([&](size_t i, Vector x_i)
     {
         if(Fluid->fluid_bflp[i] == 2)
         {
             int boundary_nerghbor_num = 0;
             _for_each_neighbor_boundaries(i, [&](size_t j, Vector x_j, size_t b_set)
             {
                 boundary_nerghbor_num++;
             });
             if(boundary_nerghbor_num == 0)
             {
                 Fluid->fluid_vp[i] = 1;
             }
         }
     });
}
void HinaPE::DFSPH_AkinciSolver::compute_density()
{
	_for_each_fluid_particle(
			[&](size_t i, Vector x_i)
			{
				real rho_i = Fluid->V[i] * Kernel::W_zero();
				_for_each_neighbor_fluid(i, [&](size_t j, Vector x_j)
				{
					rho_i += Fluid->V[j] * Kernel::W(x_i - x_j);
				});
				_for_each_neighbor_boundaries(i, [&](size_t j, Vector x_j, size_t b_set)
				{
					rho_i += Boundaries[b_set]->V[j] * Kernel::W(x_i - x_j);
				});
				Fluid->rho[i] = rho_i * FLUID_REST_DENSITY;
			});
}
void HinaPE::DFSPH_AkinciSolver::compute_factor()
{
	_for_each_fluid_particle(
			[&](size_t i, Vector x_i)
			{
				real sum_grad_p_k = 0.;
				Vector grad_p_i{0, 0, 0};
				_for_each_neighbor_fluid(i, [&](size_t j, Vector x_j)
				{
					Vector grad_p_j = -Fluid->V[j] * Kernel::gradW(x_i - x_j);
					sum_grad_p_k += grad_p_j.length2();
					grad_p_i -= grad_p_j;
				});
				_for_each_neighbor_boundaries(i, [&](size_t j, Vector x_j, size_t b_set)
				{
					Vector grad_p_j = -Boundaries[b_set]->V[j] * Kernel::gradW(x_i - x_j);
					grad_p_i -= grad_p_j;
				});
				sum_grad_p_k += grad_p_i.length2();
				if (sum_grad_p_k > 1e-6)
					Fluid->factor[i] = -1.f / sum_grad_p_k;
				else
					Fluid->factor[i] = 0;
			});
}
void HinaPE::DFSPH_AkinciSolver::divergence_solve(HinaPE::real dt)
{
	_compute_density_change();
	real inv_dt = static_cast<real>(1.) / dt;
	_for_each_fluid_particle(
			[&](size_t i, Vector x_i)
			{
				Fluid->factor[i] *= inv_dt;
			});

	constexpr size_t m_max_iterations_v = 100;
	constexpr real max_error_V = 0.1f;

	size_t m_iterations_v = 0;
	real avg_density_err = 0.0;
	while (m_iterations_v < 1 || m_iterations_v < m_max_iterations_v)
	{
		avg_density_err = _divergence_solver_iteration(dt);
		real eta = static_cast<real>(1.) / dt * max_error_V * static_cast<real>(0.01f) * FLUID_REST_DENSITY;
		if (avg_density_err <= eta)
			break;
		m_iterations_v += 1;
	}
//	std::cout << "DFSPH - iteration V: " << m_iterations_v << " Avg density err: " << avg_density_err << std::endl;

	_for_each_fluid_particle(
			[&](size_t i, Vector x_i)
			{
				Fluid->factor[i] *= dt;
			});
}
void HinaPE::DFSPH_AkinciSolver::non_pressure_force()
{
	constexpr real d = 10;
	const real diameter = 2 * FLUID_PARTICLE_RADIUS;
	const real diameter2 = diameter * diameter;
	const real kr2 = Kernel::_r * Kernel::_r;

	_for_each_fluid_particle(
			[&](size_t i, Vector x_i)
			{
				Vector dv = GRAVITY;

				_for_each_neighbor_fluid(i, [&](size_t j, Vector x_j)
				{
					// Surface Tension
					const Vector r = x_i - x_j;
					const real r2 = r.dot(r);
					const real r1 = std::sqrt(r2);
					if (r2 > diameter2)
						dv -= FLUID_SURFACE_TENSION / Fluid->m[i] * Fluid->m[j] * r * Kernel::W(r1);
					else
						dv -= FLUID_SURFACE_TENSION / Fluid->m[i] * Fluid->m[j] * r * Kernel::W(diameter);

					// Fluid Viscosity
					real v_xy = (Fluid->v[i] - Fluid->v[j]).dot(r);
					Vector f_v = d * FLUID_VISCOSITY * (Fluid->m[j] / (Fluid->rho[j])) * v_xy / (r2 + 0.01f * kr2) * Kernel::gradW(r);
					dv += f_v;
				});
				_for_each_neighbor_boundaries(i, [&](size_t j, Vector x_j, size_t b_set)
				{
					// Boundary Viscosity
					const Vector r = x_i - x_j;
					const real r2 = r.dot(r);
					real v_xy = (Fluid->v[i] - Fluid->v[j]).dot(r);
					Vector f_v = d * BOUNDARY_VISCOSITY * (BOUNDARY_REST_DENSITY[b_set] * Boundaries[b_set]->V[j] / Fluid->rho[i]) * v_xy / (r2 + 0.01f * kr2) * Kernel::gradW(r);
					dv += f_v;

					if (BOUNDARY_DYNAMICS[b_set]) // Dynamic Rigid Body
						Boundaries[b_set]->a[j] += -f_v;
				});

				Fluid->a[i] = dv;
			});
}
void HinaPE::DFSPH_AkinciSolver::predict_velocity(HinaPE::real dt)
{
	_for_each_fluid_particle(
			[&](size_t i, Vector)
			{
				Fluid->v[i] += dt * Fluid->a[i];
			});
}
void HinaPE::DFSPH_AkinciSolver::pressure_solve(HinaPE::real dt)
{
	const real dt2 = dt * dt;
	const real inv_dt2 = static_cast<real>(1.) / dt2;
	_compute_density_adv(dt);
	_for_each_fluid_particle(
			[&](size_t i, Vector x_i)
			{
				Fluid->factor[i] *= inv_dt2;
			});

	constexpr size_t m_max_iterations = 100;
	constexpr real max_error = 0.05f;

	size_t m_iterations = 0;
	real avg_density_err = 0.0;
	while (m_iterations < 1 || m_iterations < m_max_iterations)
	{
		avg_density_err = _pressure_solve_iteration(dt);
		real eta = max_error * 0.01f * FLUID_REST_DENSITY;
		if (avg_density_err <= eta)
			break;
		m_iterations += 1;
	}
//	std::cout << "DFSPH - iteration: " << m_iterations << " Avg density err: " << avg_density_err << std::endl;

	_for_each_fluid_particle(
			[&](size_t i, Vector x_i)
			{
				Fluid->factor[i] *= dt2;
			});
}
void HinaPE::DFSPH_AkinciSolver::advect(HinaPE::real dt)
{
	_for_each_fluid_particle(
			[&](size_t i, Vector)
			{
				Fluid->x[i] += dt * Fluid->v[i];
			});
}
void HinaPE::DFSPH_AkinciSolver::enforce_SDF_boundary()
{
	_for_each_fluid_particle(
			[&](size_t i, Vector)
			{
				serial_for(SDFBoundaries.size(), [&](size_t sdf_idx)
				{
					SDFBoundaries[sdf_idx]->S->resolve_collision(FLUID_PARTICLE_RADIUS, SDF_BOUNCINESS[sdf_idx], Fluid->x[i], Fluid->v[i]);
				});
			});
}
void HinaPE::DFSPH_AkinciSolver::enforce_boundary()
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

void HinaPE::DFSPH_AkinciSolver::_for_each_fluid_particle(const std::function<void(size_t i, Vector x_i)> &f)
{
	parallel_for(Fluid->size, [&](size_t i) { f(i, Fluid->x[i]); });
}
void HinaPE::DFSPH_AkinciSolver::_for_each_neighbor_fluid(size_t i, const std::function<void(size_t, Vector)> &f)
{
	NeighborBuilder.for_each_neighbor(0, 0, i, f);
}
void HinaPE::DFSPH_AkinciSolver::_for_each_neighbor_boundaries(size_t i, const std::function<void(size_t, Vector, size_t)> &f)
{
	serial_for(Boundaries.size(), [&](size_t b_set)
	{
		NeighborBuilder.for_each_neighbor(0, b_set + 1, i, [&](size_t j, Vector x_j) { f(j, x_j, b_set); });
	});
}
/*void HinaPE::DFSPH_AkinciSolver::_for_each_boundary_particle(const std::function<void(size_t, Vector)> &f) {
    serial_for(Boundaries.size(), [&](size_t b_set)
    {
        parallel_for(Boundaries[b_set]->size, [&](size_t i)
        {
            f(i, Boundaries[b_set]->x[i]);
        });
    });
}*/
void HinaPE::DFSPH_AkinciSolver::_compute_density_change()
{
	_for_each_fluid_particle(
			[&](size_t i, Vector x_i)
			{
				real density_adv = 0;
				_for_each_neighbor_fluid(i, [&](size_t j, Vector x_j)
				{
					density_adv += Fluid->V[j] * (Fluid->v[i] - Fluid->v[j]).dot(Kernel::gradW(x_i - x_j));
				});
				_for_each_neighbor_boundaries(i, [&](size_t j, Vector x_j, size_t b_set)
				{
					density_adv += Boundaries[b_set]->V[j] * (Fluid->v[i] - Boundaries[b_set]->v[j]).dot(Kernel::gradW(x_i - x_j));
				});
				Fluid->density_adv[i] = std::max(density_adv, static_cast<real>(0));
			});
}
void HinaPE::DFSPH_AkinciSolver::_compute_density_adv(real dt)
{
	_for_each_fluid_particle(
			[&](size_t i, Vector x_i)
			{
				real delta = 0;
				_for_each_neighbor_fluid(i, [&](size_t j, Vector x_j)
				{
					delta += Fluid->V[j] * (Fluid->v[i] - Fluid->v[j]).dot(Kernel::gradW(x_i - x_j));
				});
				_for_each_neighbor_boundaries(i, [&](size_t j, Vector x_j, size_t b_set)
				{
					delta += Boundaries[b_set]->V[j] * (Fluid->v[i] - Boundaries[b_set]->v[j]).dot(Kernel::gradW(x_i - x_j));
				});
				real density_adv = Fluid->rho[i] / FLUID_REST_DENSITY + dt * delta;
				Fluid->density_adv[i] = std::max(density_adv, static_cast<real>(1.));
			});
}
HinaPE::real HinaPE::DFSPH_AkinciSolver::_compute_density_error(const real offset)
{
	real density_err = 0;
	// Important! DO NOT USE PARALLEL FOR HERE
	serial_for(Fluid->size, [&](size_t i)
	{
		density_err += FLUID_REST_DENSITY * Fluid->density_adv[i] - offset;
	});
	return density_err;
}
HinaPE::real HinaPE::DFSPH_AkinciSolver::_divergence_solver_iteration(HinaPE::real dt)
{
	_divergence_solver_iteration_kernel(dt);
	_compute_density_change();
	real density_err = _compute_density_error(static_cast<real>(0.));
	return density_err / Fluid->size;
}
HinaPE::real HinaPE::DFSPH_AkinciSolver::_pressure_solve_iteration(HinaPE::real dt)
{
	_pressure_solve_iteration_kernel(dt);
	_compute_density_adv(dt);
	real density_err = _compute_density_error(static_cast<real>(FLUID_REST_DENSITY));
	return density_err / Fluid->size;
}
void HinaPE::DFSPH_AkinciSolver::_divergence_solver_iteration_kernel(real dt)
{
	// Perform Jacobi iteration
	_for_each_fluid_particle(
			[&](size_t i, Vector x_i)
			{
				real b_i = Fluid->density_adv[i];
				real k_i = b_i * Fluid->factor[i];
				Vector dv{0, 0, 0};
				_for_each_neighbor_fluid(i, [&](size_t j, Vector x_j)
				{
					real b_j = Fluid->density_adv[j];
					real k_j = b_j * Fluid->factor[j];
					real k_sum = k_i + k_j; // for multiphase fluid
					if (std::abs(k_sum) > 1e-5)
					{
						Vector grad_p_j = -Fluid->V[j] * Kernel::gradW(x_i - x_j);
						dv -= dt * k_sum * grad_p_j;
					}
				});
				_for_each_neighbor_boundaries(i, [&](size_t j, Vector x_j, size_t b_set)
				{
					if (std::abs(k_i) > 1e-5)
					{
						Vector grad_p_j = -Boundaries[b_set]->V[j] * Kernel::gradW(x_i - x_j);
						Vector vel_change = -dt * k_i * grad_p_j;
						dv += vel_change;

						if (BOUNDARY_DYNAMICS[b_set]) // Dynamic Rigid Body
							Boundaries[b_set]->a[j] += (-vel_change) / dt;
					}
				});
				Fluid->v[i] += dv;
			});
}
void HinaPE::DFSPH_AkinciSolver::_pressure_solve_iteration_kernel(real dt)
{
	// Compute pressure forces
	_for_each_fluid_particle(
			[&](size_t i, Vector x_i)
			{
				real b_i = Fluid->density_adv[i] - static_cast<real>(1.f);
				real k_i = b_i * Fluid->factor[i];
				Vector dv{0, 0, 0};
				_for_each_neighbor_fluid(i, [&](size_t j, Vector x_j)
				{
					real b_j = Fluid->density_adv[j] - static_cast<real>(1.f);
					real k_j = b_j * Fluid->factor[j];
					real k_sum = k_i + k_j; // for multiphase fluid
					if (std::abs(k_sum) > 1e-5)
					{
						Vector grad_p_j = -Fluid->V[j] * Kernel::gradW(x_i - x_j);
						dv -= dt * k_sum * grad_p_j;
					}
				});
				_for_each_neighbor_boundaries(i, [&](size_t j, Vector x_j, size_t b_set)
				{
					if (std::abs(k_i) > 1e-5)
					{
						Vector grad_p_j = -Boundaries[b_set]->V[j] * Kernel::gradW(x_i - x_j);
						Vector vel_change = -dt * k_i * grad_p_j;
						dv += vel_change;

						if (BOUNDARY_DYNAMICS[b_set]) // Dynamic Rigid Body
							Boundaries[b_set]->a[j] += (-vel_change) / dt;
					}
				});
				Fluid->v[i] += dv;
				Fluid->k[i] = k_i;
			});
}

void HinaPE::DFSPH_AkinciSolver:: compute_vorticity_n_sph() {
    _for_each_fluid_particle([&](size_t i, Vector x_i)
     {
         Vector omega{0, 0, 0};
         _for_each_neighbor_fluid(i, [&](size_t j, Vector x_j)
         {
             Vector r = x_i - x_j;
             Vector v = Fluid->v[i] - Fluid->v[j];
             omega += Fluid->V[j] * cross( v,Kernel::gradW(r));
         });
         Fluid->omega[i] = omega;
     });
}

void HinaPE::DFSPH_AkinciSolver::compute_vorticity_n1_sph() {
    _for_each_fluid_particle([&](size_t i, Vector x_i)
     {
         Vector omega{0, 0, 0};
         _for_each_neighbor_fluid(i, [&](size_t j, Vector x_j)
         {
             Vector r = x_i - x_j;
             Vector v = Fluid->v[i] - Fluid->v[j];
             omega += Fluid->V[j] * cross( v,Kernel::gradW(r));
         });
         Fluid->predict_omega[i] = omega;
     });
}

void HinaPE::DFSPH_AkinciSolver::compute_ideal_vorticity_n1_vorticity_equation(real dt) {
    fpreal viscosity = 0.05;
    // 算Dω/dt
    _for_each_fluid_particle([&](size_t i, Vector x_i)
     {
         Vector omega = Fluid->omega[i];
         // 第一项是ω·▽v
         Vector grad_v{0, 0, 0};
        _for_each_neighbor_fluid(i, [&](size_t j, Vector x_j)
        {
            Vector r = x_i - x_j;
            Vector v = Fluid->v[j] - Fluid->v[i];
            grad_v += Fluid->V[j] * v * Kernel::gradW(r);
        });
        Fluid->first_term[i] = grad_v.dot(omega);
        /*//// 这个东西感觉很有问题,但是真的没找到为什么
         Vector second_term{0, 0, 0};
         _for_each_neighbor_fluid(i, [&](size_t j, Vector x_j)
         {
             Vector r = x_i - x_j;
             fpreal r2 = r.length() * r.length();
             Vector omega_ij = omega - Fluid->omega[j];
             second_term += Fluid->V[j] * (dot(omega_ij, r) / (r2 + 0.01 * FLUID_KERNAL_RADIUS * FLUID_KERNAL_RADIUS)) * Kernel::gradW(r);
         });
         Fluid->second_term[i] = viscosity * 2 * (3 + 2) * second_term;
         Fluid->omega[i] += (Fluid->first_term[i] + Fluid->second_term[i]) * dt;*/
/*        if(Fluid->omega[i].x() > 1 || Fluid->omega[i].y() > 1 || Fluid->omega[i].z() > 1 || Fluid->omega[i].x() < -1 || Fluid->omega[i].y() < -1 || Fluid->omega[i].z() < -1)
            Fluid->omega[i] = 0;*/
     });
    _for_each_fluid_particle([&](size_t i, Vector x_i)
     {
         // 第二项是v·▽2ω
         Vector omega = Fluid->omega[i];
         Vector second_term{0, 0, 0};
         _for_each_neighbor_fluid(i, [&](size_t j, Vector x_j)
        {
            Vector r = x_i - x_j;
            fpreal r2 = r.length() * r.length();
            Vector omega_ij = omega - Fluid->omega[j];
            second_term += Fluid->V[j] * (dot(omega_ij, r) / (r2 + 0.01 * FLUID_KERNAL_RADIUS * FLUID_KERNAL_RADIUS)) * Kernel::gradW(r);
        });
        Fluid->second_term[i] = viscosity * 2 * (3 + 2) * second_term;
     });
    _for_each_fluid_particle([&](size_t i, Vector x_i)
     {
         Fluid->omega[i] += (Fluid->first_term[i] + Fluid->second_term[i]) * dt;
     });

}

void HinaPE::DFSPH_AkinciSolver::compute_vorticity_dissipation() {
    _for_each_fluid_particle([&](size_t i, Vector x_i)
     {
         Fluid->omega_delta[i] = Fluid->omega[i] - Fluid->predict_omega[i];
     });
}

void HinaPE::DFSPH_AkinciSolver::compute_stream_function() {
    _for_each_fluid_particle([&](size_t i, Vector x_i)
     {
         Vector psi{0, 0, 0};
        _for_each_neighbor_fluid(i, [&](size_t j, Vector x_j) {
            Vector xij = x_i - x_j;
            if(xij.length() <= FLUID_KERNAL_RADIUS)
                psi += (Fluid->omega_delta[j] * Fluid->V[j]) / xij.length();
        });
         Fluid->psi[i] = psi / (4 * M_PI);
     });
}

void HinaPE::DFSPH_AkinciSolver::compute_vorticity_velocity() {
    _for_each_fluid_particle([&](size_t i, Vector x_i)
     {
        Vector vorticity_velocity{0, 0, 0};
        _for_each_neighbor_fluid(i, [&](size_t j, Vector x_j) {
            Vector xij = x_i - x_j;
            Vector fsi_ij = Fluid->psi[i] - Fluid->psi[j];
            vorticity_velocity += Fluid->V[j] * cross(fsi_ij, Kernel::gradW(xij));
        });
        Fluid->v[i] += BETA * vorticity_velocity;
     });
}

void HinaPE::DFSPH_AkinciSolver::clearMarkedParticles() {
    _for_each_fluid_particle([&](size_t i, Vector x_i)
     {
         Fluid->fluid_bflp[i] = 0;
         Fluid->fluid_vp[i] = 0;
     });

    serial_for(Boundaries.size(), [&](size_t b_set)
    {
        parallel_for(Boundaries[b_set]->size, [&](size_t i)
        {
            Boundaries[b_set]->boundary_sp[i] = 0;
        });
    });
}

















