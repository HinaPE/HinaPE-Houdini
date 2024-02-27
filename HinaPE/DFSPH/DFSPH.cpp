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

//void HinaPE::parallel_for(size_t n, const std::function<void(size_t)> &f) { tbb::parallel_for(size_t(0), n, [&](size_t i) { f(i); }); }
void HinaPE::parallel_for(size_t n, const std::function<void(size_t)> &f) { serial_for(n, f); }
void HinaPE::serial_for(size_t n, const std::function<void(size_t)> &f) { for (size_t i = 0; i < n; ++i) { f(i); }}
HinaPE::DFSPHSolverCPU::DFSPHSolverCPU(real radius) : NeighborBuilder(radius), Kernel(radius) {}
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
	enforce_boundary();
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
			volume += Kernel.kernel(0); // self is also a neighbor
			NeighborBuilder.for_each_neighbor(boundary_set_idx + 1, boundary_set_idx + 1, boundary_pt_idx, [&](size_t j, Vector x_j)
			{
				volume += Kernel.kernel((x_i - x_j).length());
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
		rho += m_i * Kernel.kernel(0.);
		for_each_neighbor_fluid(
				i, [&](size_t j, Vector x_j)
				{
					real m_j = Fluid.m[j];
					rho += m_j * Kernel.kernel((x_i - x_j).length());
				});
		for_each_neighbor_static_boundary(
				i, [&](size_t j, Vector x_j, size_t boundary_idx)
				{
					real m_j = StaticBoundaries[boundary_idx].m[j];
					rho += m_j * Kernel.kernel((x_i - x_j).length());
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
					Vector grad = m_j * Kernel.gradient(x_i - x_j);
					grad_sum += grad;
					grad_square_sum += grad.length2();
				});
		for_each_neighbor_static_boundary(
				i, [&](size_t j, Vector x_j, size_t boundary_idx)
				{
					real m_j = StaticBoundaries[boundary_idx].m[j];
					Vector grad = m_j * Kernel.gradient(x_i - x_j);
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
							Vector grad = V_j * (k_i + k_j) * Kernel.gradient(x_i - x_j);
							Fluid.v[i] -= dt * grad;
						}
					});
			for_each_neighbor_static_boundary(
					i, [&](size_t j, Vector x_j, size_t boundary_idx)
					{
						real V_j = StaticBoundaries[boundary_idx].V[j];
						Vector grad = V_j * Kernel.gradient(x_i - x_j);

						Vector v_change = dt * 1.f * k_i * grad;
						Fluid.v[i] -= v_change;
						// TODO: if is dynamic rigid body, handle accelerations
					});
		});
		compute_density_adv(dt);
		compute_avg_density_error_den();
//		std::cout << "den::Iter::" << iter << "error" << Fluid.avg_density_error << std::endl;
		if (Fluid.avg_density_error < eta)
			break;
		++iter;
	}
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
							Vector grad = V_j * (k_i + k_j) * Kernel.gradient(x_i - x_j);
							dv -= dt * grad;
						}
					});
			for_each_neighbor_static_boundary(
					i, [&](size_t j, Vector x_j, size_t boundary_idx)
					{
						real V_j = StaticBoundaries[boundary_idx].V[j];
						Vector grad = V_j * Kernel.gradient(x_i - x_j);

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
					delta += V_j * (v_i - v_j).dot(Kernel.gradient(x_i - x_j));
				});

		for_each_neighbor_static_boundary(
				i, [&](size_t j, Vector x_j, size_t boundary_idx)
				{
					Vector v_j = StaticBoundaries[boundary_idx].v[j];
					real V_j = StaticBoundaries[boundary_idx].V[j];
					delta += V_j * (v_i - v_j).dot(Kernel.gradient(x_i - x_j));
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
					density_adv += V_j * (v_i - v_j).dot(Kernel.gradient(x_i - x_j));
				});
		for_each_neighbor_static_boundary(
				i, [&](size_t j, Vector x_j, size_t boundary_idx)
				{
					Vector v_j = StaticBoundaries[boundary_idx].v[j];
					real V_j = StaticBoundaries[boundary_idx].V[j];
					density_adv += V_j * (v_i - v_j).dot(Kernel.gradient(x_i - x_j));
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
	real bound = .99f;
	real damping = .5f;
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
