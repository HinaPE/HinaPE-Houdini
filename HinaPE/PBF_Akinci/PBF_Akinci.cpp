#include "PBF_Akinci.h"

#include <numeric>
#include <utility>
#include <execution>

HinaPE::PBF_AkinciSolver::PBF_AkinciSolver(HinaPE::real _r, HinaPE::Vector _b)
		: NeighborBuilder(_r), MaxBound(_b / 2.), NeighborBuilderInited(false)
{
	Kernel::set_radius(_r);
	Fluid = std::make_shared<PBF_AkinciFluid>();
}
void HinaPE::PBF_AkinciSolver::Solve(HinaPE::real dt)
{
	resize();

	predict_x(dt);
	build_neighbors();
	pbf_iteration(dt);
	update_v(dt);
	compute_density();
	non_pressure_force();
	update_x(dt);
	enforce_boundary();
}
void HinaPE::PBF_AkinciSolver::resize()
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

		Fluid->p_x.resize(Fluid->size);
	}
}
void HinaPE::PBF_AkinciSolver::predict_x(real dt)
{
	std::fill(Fluid->a.begin(), Fluid->a.end(), GRAVITY);
	_for_each_fluid_particle(
			[&](size_t i, Vector)
			{
				Fluid->v[i] += dt * Fluid->a[i];
				Fluid->p_x[i] = Fluid->x[i] + dt * Fluid->v[i];
			});
}
void HinaPE::PBF_AkinciSolver::build_neighbors()
{
	if (!NeighborBuilderInited)
	{
		real d = static_cast<real>(2.f) * FLUID_PARTICLE_RADIUS;
		std::fill(Fluid->V.begin(), Fluid->V.end(), static_cast<real>(.8f) * d * d * d);
		std::transform(Fluid->V.begin(), Fluid->V.end(), Fluid->m.begin(), [&](real V) { return V * FLUID_REST_DENSITY; });

		std::vector<VectorArrayCPU *> x_sets;
		x_sets.emplace_back(&Fluid->p_x);
		NeighborBuilder.init(x_sets);
		NeighborBuilderInited = true;
	} else
		NeighborBuilder.resize_set(0, &Fluid->p_x); // if new fluid particles is generated, update fluid set

	NeighborBuilder.update_set(0); // 0 means fluid set
	NeighborBuilder.build();

	_for_each_fluid_particle(
			[&](size_t i, Vector x_i)
			{
				Fluid->neighbor_this[i] = NeighborBuilder.n_neighbors(0, 0, i);
				Fluid->neighbor_others[i] = 0;
			});
}
void HinaPE::PBF_AkinciSolver::pbf_iteration(HinaPE::real dt)
{
	real avg_density_err = 0.f;
	for (int i = 0; i < PBF_ITERATION; ++i)
	{
		avg_density_err = _pressure_solver_iteration(dt);

		const real eta = MAX_DENSITY_ERROR * static_cast<real>(0.01) * FLUID_REST_DENSITY;  // TODO: make max_error parameterized
		if (avg_density_err <= eta)
			break;
	}
}
void HinaPE::PBF_AkinciSolver::update_v(real dt)
{
	_for_each_fluid_particle(
			[&](size_t i, Vector)
			{
				Fluid->v[i] = (Fluid->p_x[i] - Fluid->x[i]) / dt;
			});
}
void HinaPE::PBF_AkinciSolver::compute_density()
{
	_for_each_fluid_particle(
			[&](size_t i, Vector x_i)
			{
				real rho_i = Fluid->V[i] * Kernel::W_zero();
				_for_each_neighbor_fluid(i, [&](size_t j, Vector x_j)
				{
					rho_i += Fluid->V[j] * Kernel::W(x_i - x_j);
				});
				Fluid->rho[i] = rho_i * FLUID_REST_DENSITY;
			});
}
void HinaPE::PBF_AkinciSolver::non_pressure_force()
{

}
void HinaPE::PBF_AkinciSolver::update_x(HinaPE::real dt)
{
	_for_each_fluid_particle(
			[&](size_t i, Vector)
			{
				Fluid->x[i] = Fluid->p_x[i];
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
	parallel_for(Fluid->size, [&](size_t i) { f(i, Fluid->p_x[i]); });
}
void HinaPE::PBF_AkinciSolver::_for_each_neighbor_fluid(size_t i, const std::function<void(size_t, Vector)> &f)
{
	NeighborBuilder.for_each_neighbor(0, 0, i, f);
}
HinaPE::real HinaPE::PBF_AkinciSolver::_pressure_solver_iteration(HinaPE::real dt)
{
	const real inv_dt = static_cast<real>(1.f) / dt;
	const real inv_dt2 = inv_dt * inv_dt;
	return 0;
}
