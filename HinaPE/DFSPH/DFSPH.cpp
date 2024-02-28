#include "DFSPH.h"
#include <tbb/tbb.h>

void resize(HinaPE::DFSPHFluidCPU &t)
{
	t.x.resize(t.size);
	t.v.resize(t.size);
	t.f.resize(t.size);
	t.m.resize(t.size);
	t.V.resize(t.size);
	t.rho.resize(t.size);
	t.alpha.resize(t.size);
	t.kappa_density.resize(t.size);
	t.kappa_divergence.resize(t.size);
	t.rho_adv.resize(t.size);
	t.d_rho.resize(t.size);

	t.neighbor_this.resize(t.size);
	t.neighbor_others.resize(t.size);
}

void resize(HinaPE::AkinciBoundaryCPU &t)
{
	t.x.resize(t.size);
	t.v.resize(t.size);
	t.m.resize(t.size);
	t.V.resize(t.size);
	t.rho.resize(t.size);

	t.neighbor_this.resize(t.size);
	t.neighbor_others.resize(t.size);
}

HinaPE::DFSPHSolverCPU::DFSPHSolverCPU(real radius) : NeighborBuilder(radius)
{
	CubicKernel::set_radius(radius);
}
void HinaPE::DFSPHSolverCPU::Init()
{
	Fluid.size = Fluid.x.size();
	resize(Fluid);
	for (auto &static_boundary: StaticBoundaries)
	{
		static_boundary.size = static_boundary.x.size();
		resize(static_boundary);
	}

	std::vector<CPUVectorArray *> x_sets;
	x_sets.emplace_back(&Fluid.x);
	for (auto &static_boundary: StaticBoundaries)
		x_sets.emplace_back(&static_boundary.x);
	NeighborBuilder.init(x_sets);

	build_neighbors();
	compute_akinci_volume_mass();
	compute_density();
}
void HinaPE::DFSPHSolverCPU::Solve(HinaPE::real dt)
{
	compute_non_pressure_force();
	advect_vel(dt);
	correct_density_error(dt);
	advect_pos(dt);
//	enforce_boundary();
	build_neighbors();
	compute_density();
	compute_alpha();
	correct_divergence_error(dt);
}
void HinaPE::DFSPHSolverCPU::build_neighbors(bool new_fluid_particles)
{
	if (new_fluid_particles)
		NeighborBuilder.resize_set(0, &Fluid.x);
	NeighborBuilder.update_set(0);
	NeighborBuilder.build();

	serial_for(Fluid.size, [&](size_t fluid_pt_idx)
	{
		Fluid.neighbor_this[fluid_pt_idx] = NeighborBuilder.n_neighbors(0, 0, fluid_pt_idx);
		Fluid.neighbor_others[fluid_pt_idx] = 0;
		serial_for(StaticBoundaries.size(), [&](size_t boundary_set_idx)
		{
			Fluid.neighbor_others[fluid_pt_idx] += NeighborBuilder.n_neighbors(0, boundary_set_idx + 1, fluid_pt_idx);
		});
	});

	serial_for(StaticBoundaries.size(), [&](size_t boundary_set_idx)
	{
		serial_for(StaticBoundaries[boundary_set_idx].size, [&](size_t boundary_pt_idx)
		{
			StaticBoundaries[boundary_set_idx].neighbor_this[boundary_pt_idx] = NeighborBuilder.n_neighbors(boundary_set_idx + 1, boundary_set_idx + 1, boundary_pt_idx);
			StaticBoundaries[boundary_set_idx].neighbor_others[boundary_pt_idx] = NeighborBuilder.n_neighbors(boundary_set_idx + 1, 0, boundary_pt_idx);
			serial_for(StaticBoundaries.size(), [&](size_t other_boundary_set_idx)
			{
				if (other_boundary_set_idx != boundary_set_idx)
					StaticBoundaries[boundary_set_idx].neighbor_others[boundary_pt_idx] += NeighborBuilder.n_neighbors(boundary_set_idx + 1, other_boundary_set_idx + 1, boundary_pt_idx);
			});
		});
	});
}
void HinaPE::DFSPHSolverCPU::for_each_neighbor_fluid(size_t pt_idx, const std::function<void(size_t, Vector)> &f) const { NeighborBuilder.for_each_neighbor(0, 0, pt_idx, f); }
void HinaPE::DFSPHSolverCPU::for_each_neighbor_static_boundary(size_t pt_idx, const std::function<void(size_t, Vector, size_t)> &f) const
{
	serial_for(StaticBoundaries.size(), [&](size_t i)
	{
		NeighborBuilder.for_each_neighbor(0, i + 1, pt_idx, [&](size_t j, Vector x_j) { f(j, x_j, i); });
	});
}

void HinaPE::DFSPHSolverCPU::advect_pos(HinaPE::real dt) { parallel_for(Fluid.size, [&](size_t i) { Fluid.x[i] += Fluid.v[i] * dt; }); }
void HinaPE::DFSPHSolverCPU::advect_vel(HinaPE::real dt) { parallel_for(Fluid.size, [&](size_t i) { Fluid.v[i] += Fluid.f[i] / Fluid.m[i] * dt; }); }
void HinaPE::DFSPHSolverCPU::compute_non_pressure_force()
{
	parallel_for(Fluid.size, [&](size_t i) { Fluid.f[i] = Fluid.m[i] * Vector(0, -9.8f, 0); });
	parallel_for(Fluid.size, [&](size_t i) { Fluid.v[i] *= 0.99f; });
}
void HinaPE::DFSPHSolverCPU::compute_akinci_volume_mass()
{
	serial_for(StaticBoundaries.size(), [&](size_t boundary_set_idx)
	{
		serial_for(StaticBoundaries[boundary_set_idx].size, [&](size_t boundary_pt_idx)
		{
			real volume = 0;
			real mass = 0;
			Vector x_i = StaticBoundaries[boundary_set_idx].x[boundary_pt_idx];
			real rho0 = StaticBoundaries[boundary_set_idx].rho[boundary_pt_idx];
			volume += CubicKernel::W_zero(); // self is also a neighbor
			NeighborBuilder.for_each_neighbor(boundary_set_idx + 1, boundary_set_idx + 1, boundary_pt_idx, [&](size_t j, Vector x_j)
			{
				volume += CubicKernel::W(x_i - x_j);
			});
			volume = 1. / volume;
			mass = volume * rho0;
			StaticBoundaries[boundary_set_idx].V[boundary_pt_idx] = volume;
			StaticBoundaries[boundary_set_idx].m[boundary_pt_idx] = mass;
		});
	});
}
void HinaPE::DFSPHSolverCPU::compute_density()
{
	parallel_for(Fluid.size, [&](size_t i)
	{
		real rho = 0.;
		Vector x_i = Fluid.x[i];
		real m_i = Fluid.m[i];
		rho += m_i * CubicKernel::W(0.);
		for_each_neighbor_fluid(
				i, [&](size_t j, Vector x_j)
				{
					real m_j = Fluid.m[j];
					rho += m_j * CubicKernel::W(x_i - x_j);
				});
		for_each_neighbor_static_boundary(
				i, [&](size_t j, Vector x_j, size_t boundary_idx)
				{
					real m_j = StaticBoundaries[boundary_idx].m[j];
					rho += m_j * CubicKernel::W(x_i - x_j);
				});
		Fluid.rho[i] = rho;
		Fluid.V[i] = Fluid.m[i] / rho;
	});
}
void HinaPE::DFSPHSolverCPU::compute_alpha()
{
	parallel_for(Fluid.size, [&](size_t i)
	{
		real alpha = 0;
		fpreal grad_square_sum = 0;
		fpreal grad_sum_square = 0;
		UT_Vector3 grad_sum = {0, 0, 0};

		Vector x_i = Fluid.x[i];
		real m_i = Fluid.m[i];
		real rho_i = Fluid.rho[i];
		for_each_neighbor_fluid(
				i, [&](size_t j, Vector x_j)
				{
					real m_j = Fluid.m[j];
					Vector grad = m_j * CubicKernel::gradW(x_i - x_j);
					grad_sum += grad;
					grad_square_sum += grad.length2();
				});
		for_each_neighbor_static_boundary(
				i, [&](size_t j, Vector x_j, size_t boundary_idx)
				{
					real m_j = StaticBoundaries[boundary_idx].m[j];
					Vector grad = m_j * CubicKernel::gradW(x_i - x_j);
					grad_sum += grad;
//					grad_square_sum += grad.length2(); // no need for akinci boundary
				});
		grad_sum_square = grad_sum.length2();
		real denominator = grad_square_sum + grad_sum_square;
		if (denominator > 1e-6)
			alpha = rho_i / denominator;
		else
			alpha = 0;
		Fluid.alpha[i] = alpha;
	});
}
void HinaPE::DFSPHSolverCPU::correct_density_error(real dt)
{
	compute_density_adv(dt);

	int iter = 0;
	int max_iter = 100;
	real eta = 0.05 * 0.01 * FluidRestDensity;
	while (iter < max_iter)
	{
		// pressure solve iteration kernel
		parallel_for(Fluid.size, [&](size_t i)
		{
			Vector x_i = Fluid.x[i];
			real b_i = Fluid.rho_adv[i] - 1.f;
			real k_i = b_i * Fluid.alpha[i] / (dt * dt);
			for_each_neighbor_fluid(
					i, [&](size_t j, Vector x_j)
					{
						real b_j = Fluid.rho_adv[j] - 1.f;
						real k_j = b_j * Fluid.alpha[j] / (dt * dt);
						real V_j = Fluid.V[j];
						if (std::abs(k_i + k_j) > 1e-5)
						{
							Vector grad = V_j * (k_i + k_j) * CubicKernel::gradW(x_i - x_j);
							Fluid.v[i] -= dt * grad;
						}
					});
			for_each_neighbor_static_boundary(
					i, [&](size_t j, Vector x_j, size_t boundary_idx)
					{
						real V_j = StaticBoundaries[boundary_idx].V[j];
						Vector grad = V_j * CubicKernel::gradW(x_i - x_j);

						Vector v_change = dt * 1.f * k_i * grad;
						Fluid.v[i] -= v_change;
						// TODO: if is dynamic rigid body, handle accelerations
					});
		});
		compute_density_adv(dt);
		compute_avg_density_error_den();
		if (Fluid.avg_density_error < eta)
			break;
		++iter;
	}
	std::cout << "den::Iter::" << iter << "error" << Fluid.avg_density_error << std::endl;
}
void HinaPE::DFSPHSolverCPU::correct_divergence_error(real dt)
{
	compute_density_change(dt);
	int iter = 0;
	int max_iter = 100;
	real eta = 0.1 * 0.01 * FluidRestDensity / dt;
	while (iter < max_iter)
	{
		// divergence solver iteration (Perform Jacobi iteration)
		parallel_for(Fluid.size, [&](size_t i)
		{
			Vector x_i = Fluid.x[i];
			real b_i = Fluid.rho_adv[i];
			real k_i = b_i * Fluid.alpha[i] / dt;
			Vector dv{0, 0, 0};
			for_each_neighbor_fluid(
					i, [&](size_t j, Vector x_j)
					{
						real b_j = Fluid.rho_adv[j];
						real k_j = b_j * Fluid.alpha[j] / dt;
						real V_j = Fluid.V[j];
						if (std::abs(k_i + k_j) > 1e-5)
						{
							Vector grad = V_j * (k_i + k_j) * CubicKernel::gradW(x_i - x_j);
							dv -= dt * grad;
						}
					});
			for_each_neighbor_static_boundary(
					i, [&](size_t j, Vector x_j, size_t boundary_idx)
					{
						real V_j = StaticBoundaries[boundary_idx].V[j];
						Vector grad = V_j * CubicKernel::gradW(x_i - x_j);

						Vector v_change = dt * 1.f * k_i * grad;
						dv -= v_change;
						// TODO: if is dynamic rigid body, handle accelerations
					});
			Fluid.v[i] += dv;
		});
		compute_density_change(dt);
		compute_avg_density_error_div();
//		std::cout << "div::Iter::" << iter << "error" << Fluid.avg_density_error << std::endl;
		if (Fluid.avg_density_error < eta)
			break;
		++iter;
	}
	std::cout << "div::Iter::" << iter << "error" << Fluid.avg_density_error << std::endl;
}
void HinaPE::DFSPHSolverCPU::compute_density_adv(real dt)
{
	// Compute rho_adv
	parallel_for(Fluid.size, [&](size_t i)
	{
		real delta = 0;
		Vector x_i = Fluid.x[i];
		Vector v_i = Fluid.v[i];
		real V_i = Fluid.V[i];
		for_each_neighbor_fluid(
				i, [&](size_t j, Vector x_j)
				{
					Vector v_j = Fluid.v[j];
					real V_j = Fluid.V[j];
					delta += V_j * (v_i - v_j).dot(CubicKernel::gradW(x_i - x_j));
				});

		for_each_neighbor_static_boundary(
				i, [&](size_t j, Vector x_j, size_t boundary_idx)
				{
					Vector v_j = StaticBoundaries[boundary_idx].v[j];
					real V_j = StaticBoundaries[boundary_idx].V[j];
					delta += V_j * (v_i - v_j).dot(CubicKernel::gradW(x_i - x_j));
				});
		Fluid.rho_adv[i] = (Fluid.rho[i] / FluidRestDensity) + dt * delta;
		Fluid.rho_adv[i] = std::max(Fluid.rho_adv[i], 1.f);
	});
}
void HinaPE::DFSPHSolverCPU::compute_density_change(real dt)
{
	// Compute velocity of density change
	parallel_for(Fluid.size, [&](size_t i)
	{
		real density_adv = 0;
		Vector x_i = Fluid.x[i];
		Vector v_i = Fluid.v[i];
		real V_i = Fluid.V[i];
		for_each_neighbor_fluid(
				i, [&](size_t j, Vector x_j)
				{
					Vector v_j = Fluid.v[j];
					real V_j = Fluid.V[j];
					density_adv += V_j * (v_i - v_j).dot(CubicKernel::gradW(x_i - x_j));
				});
		for_each_neighbor_static_boundary(
				i, [&](size_t j, Vector x_j, size_t boundary_idx)
				{
					Vector v_j = StaticBoundaries[boundary_idx].v[j];
					real V_j = StaticBoundaries[boundary_idx].V[j];
					density_adv += V_j * (v_i - v_j).dot(CubicKernel::gradW(x_i - x_j));
				});
		density_adv = std::max(density_adv, 0.f); // only correct positive divergence
		Fluid.rho_adv[i] = density_adv;
	});
}
void HinaPE::DFSPHSolverCPU::compute_avg_density_error_den()
{
	real density_error = 0;
	serial_for(Fluid.size, [&](size_t i)
	{
		density_error += FluidRestDensity * (Fluid.rho_adv[i] - 1.f);
	});
	Fluid.avg_density_error = density_error / Fluid.size;
}
void HinaPE::DFSPHSolverCPU::compute_avg_density_error_div()
{
	real density_error = 0;
	serial_for(Fluid.size, [&](size_t i)
	{
		density_error += FluidRestDensity * Fluid.rho_adv[i];
	});
	Fluid.avg_density_error = density_error / Fluid.size;
}
void HinaPE::DFSPHSolverCPU::enforce_boundary()
{
	real damping = .9f;
	parallel_for(Fluid.size, [&](size_t i)
	{
		Vector x_i = Fluid.x[i];

		if (x_i.x() > bound)
		{
			Fluid.x[i].x() = bound;
			Fluid.v[i].x() *= -damping;
		}
		if (x_i.x() < -bound)
		{
			Fluid.x[i].x() = -bound;
			Fluid.v[i].x() *= -damping;
		}
		if (x_i.y() > bound)
		{
			Fluid.x[i].y() = bound;
			Fluid.v[i].y() *= -damping;
		}
		if (x_i.y() < -bound)
		{
			Fluid.x[i].y() = -bound;
			Fluid.v[i].y() *= -damping;
		}
		if (x_i.z() > bound)
		{
			Fluid.x[i].z() = bound;
			Fluid.v[i].z() *= -damping;
		}
		if (x_i.z() < -bound)
		{
			Fluid.x[i].z() = -bound;
			Fluid.v[i].z() *= -damping;
		}
	});
}

HinaPE::DFSPHSolverSPlisHSPlasH::DFSPHSolverSPlisHSPlasH(HinaPE::real radius) : NeighborBuilder(radius)
{
	CubicKernel::set_radius(radius);
}
void HinaPE::DFSPHSolverSPlisHSPlasH::build_neighbors()
{
	NeighborBuilder.update_set(0);
	NeighborBuilder.build();

	serial_for(Fluid.size, [&](size_t fluid_pt_idx)
	{
		Fluid.neighbor_this[fluid_pt_idx] = NeighborBuilder.n_neighbors(0, 0, fluid_pt_idx);
		Fluid.neighbor_others[fluid_pt_idx] = 0;
		serial_for(StaticBoundaries.size(), [&](size_t boundary_set_idx)
		{
			Fluid.neighbor_others[fluid_pt_idx] += NeighborBuilder.n_neighbors(0, boundary_set_idx + 1, fluid_pt_idx);
		});
	});

	serial_for(StaticBoundaries.size(), [&](size_t boundary_set_idx)
	{
		serial_for(StaticBoundaries[boundary_set_idx].size, [&](size_t boundary_pt_idx)
		{
			StaticBoundaries[boundary_set_idx].neighbor_this[boundary_pt_idx] = NeighborBuilder.n_neighbors(boundary_set_idx + 1, boundary_set_idx + 1, boundary_pt_idx);
			StaticBoundaries[boundary_set_idx].neighbor_others[boundary_pt_idx] = NeighborBuilder.n_neighbors(boundary_set_idx + 1, 0, boundary_pt_idx);
			serial_for(StaticBoundaries.size(), [&](size_t other_boundary_set_idx)
			{
				if (other_boundary_set_idx != boundary_set_idx)
					StaticBoundaries[boundary_set_idx].neighbor_others[boundary_pt_idx] += NeighborBuilder.n_neighbors(boundary_set_idx + 1, other_boundary_set_idx + 1, boundary_pt_idx);
			});
		});
	});
}
void HinaPE::DFSPHSolverSPlisHSPlasH::for_each_neighbor_fluid(size_t pt_idx, const std::function<void(size_t, Vector)> &f) const { NeighborBuilder.for_each_neighbor(0, 0, pt_idx, f); }
void HinaPE::DFSPHSolverSPlisHSPlasH::for_each_neighbor_static_boundary(size_t pt_idx, const std::function<void(size_t, Vector, size_t)> &f) const
{
	serial_for(StaticBoundaries.size(), [&](size_t i)
	{
		NeighborBuilder.for_each_neighbor(0, i + 1, pt_idx, [&](size_t j, Vector x_j) { f(j, x_j, i); });
	});
}
void HinaPE::DFSPHSolverSPlisHSPlasH::precompute_values()
{
	// TODO: for better performance
}
void HinaPE::DFSPHSolverSPlisHSPlasH::compute_densities()
{
	parallel_for(Fluid.size, [&](size_t i)
	{
		Vector x_i = Fluid.x[i];
		real rho = Fluid.V[i] * CubicKernel::W_zero();
		for_each_neighbor_fluid(
				i, [&](size_t j, Vector x_j)
				{
					rho += Fluid.V[j] * CubicKernel::W((x_i - x_j));
				});
		for_each_neighbor_static_boundary(
				i, [&](size_t j, Vector x_j, size_t boundary_idx)
				{
					rho += StaticBoundaries[boundary_idx].V[j] * CubicKernel::W((x_i - x_j));
				});
		Fluid.rho[i] = rho * FluidRestDensity;
	});
}
void HinaPE::DFSPHSolverSPlisHSPlasH::compute_DFSPH_factors()
{
	parallel_for(Fluid.size, [&](size_t i)
	{
		Vector x_i = Fluid.x[i];
		real sum_grad_p_k = 0.f;
		Vector grad_p_i{0, 0, 0};
		for_each_neighbor_fluid(
				i, [&](size_t j, Vector x_j)
				{
					real V_j = Fluid.V[j];
					const Vector grad_p_j = -V_j * CubicKernel::gradW(x_i - x_j);
					sum_grad_p_k += grad_p_j.length2();
					grad_p_i -= grad_p_j;
				});
		for_each_neighbor_static_boundary(
				i, [&](size_t j, Vector x_j, size_t boundary_idx)
				{
					real V_j = StaticBoundaries[boundary_idx].V[j];
					const Vector grad_p_j = -V_j * CubicKernel::gradW(x_i - x_j);
					grad_p_i -= grad_p_j;
				});
		sum_grad_p_k += grad_p_i.length2();

		// Compute factor as: factor_i = -1 / (a_ii * rho_i^2)
		// where a_ii is the diagonal entry of the linear system
		// for the pressure A * p = source term
		if (sum_grad_p_k > eps)
			Fluid.factor[i] = 1.f / sum_grad_p_k;
		else
			Fluid.factor[i] = 0.f;
	});
}
void HinaPE::DFSPHSolverSPlisHSPlasH::divergence_solve(real dt)
{
	const unsigned int max_iter = 100;
	const real max_error_v = .1f; // percentage
}
void HinaPE::DFSPHSolverSPlisHSPlasH::compute_non_pressure_force()
{

}
void HinaPE::DFSPHSolverSPlisHSPlasH::update_CFL_timestep()
{

}
void HinaPE::DFSPHSolverSPlisHSPlasH::advect_vel()
{

}
void HinaPE::DFSPHSolverSPlisHSPlasH::pressure_solve()
{

}
void HinaPE::DFSPHSolverSPlisHSPlasH::advect_pos()
{

}
