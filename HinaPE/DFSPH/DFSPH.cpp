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

void HinaPE::parallel_for(size_t n, const std::function<void(size_t)> &f) { tbb::parallel_for(size_t(0), n, [&](size_t i) { f(i); }); }
//void HinaPE::parallel_for(size_t n, const std::function<void(size_t)> &f) { serial_for(n, f); }
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
	compute_density();
}
void HinaPE::DFSPHSolverCPU::Solve(HinaPE::real dt)
{
	compute_non_pressure_force();
	advect_vel(dt);
//	correct_density_error(dt);
	advect_pos(dt);
	build_neighbors();
	compute_density();
	compute_alpha();
//	correct_divergence_error(dt);
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
	for (size_t i = 0; i < StaticBoundaries.size(); ++i)
		NeighborBuilder.for_each_neighbor(0, i + 1, pt_idx, [&](size_t j, Vector x) { f(j, x, i); });
}

void HinaPE::DFSPHSolverCPU::advect_pos(HinaPE::real dt) { parallel_for(Fluid.size, [&](size_t i) { Fluid.x[i] += Fluid.v[i] * dt; }); }
void HinaPE::DFSPHSolverCPU::advect_vel(HinaPE::real dt) { parallel_for(Fluid.size, [&](size_t i) { Fluid.v[i] += Fluid.f[i] / Fluid.m[i] * dt; }); }
void HinaPE::DFSPHSolverCPU::compute_non_pressure_force() { parallel_for(Fluid.size, [&](size_t i) { Fluid.f[i] = Fluid.m[i] * Vector(0, -9.8f, 0); }); }
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
					grad_square_sum += grad.length2();
				});
		grad_sum_square = grad_sum.length2();
		real denominator = grad_square_sum + grad_sum_square;
		if (denominator < 1e-6)
			denominator = 1e-6;
		alpha = rho_i / denominator;
		Fluid.alpha[i] = alpha;
	});
}
void HinaPE::DFSPHSolverCPU::compute_kappa_density(real dt)
{
	parallel_for(Fluid.size, [&](size_t i)
	{
		real alpha = Fluid.alpha[i];
		real rho_star = Fluid.rho_adv[i];
		real kappa = (rho_star - FluidRestDensity) * alpha / (dt * dt);
		Fluid.kappa_density[i] = kappa;
	});
}
void HinaPE::DFSPHSolverCPU::compute_kappa_divergence(real dt)
{
	parallel_for(Fluid.size, [&](size_t i)
	{
		real alpha = Fluid.alpha[i];
		real d_rho = Fluid.d_rho[i];
		real kappa = d_rho * alpha / dt;
		Fluid.kappa_divergence[i] = kappa;
	});
}
void HinaPE::DFSPHSolverCPU::compute_density_error(real dt)
{
	Fluid.avg_density_adv = 0;
	Fluid.avg_d_density = 0;
	parallel_for(Fluid.size, [&](size_t i)
	{
		real d_density = 0;
		real density_adv = 0;
		Vector x_i = Fluid.x[i];
		Vector v_i = Fluid.v[i];
		real m_i = Fluid.m[i];
		real rho = Fluid.rho[i];
		for_each_neighbor_fluid(
				i, [&](size_t j, Vector x_j)
				{
					Vector v_j = Fluid.v[j];
					real m_j = Fluid.m[j];
					d_density += m_j * (v_i - v_j).dot(Kernel.gradient(x_i - x_j));
				});
		for_each_neighbor_static_boundary(
				i, [&](size_t j, Vector x_j, size_t boundary_idx)
				{
					Vector v_j = StaticBoundaries[boundary_idx].v[j];
					real m_j = StaticBoundaries[boundary_idx].m[j];
					d_density += m_j * (v_i - v_j).dot(Kernel.gradient(x_i - x_j));
				});
		density_adv = rho + dt * d_density;
		density_adv = std::max(density_adv, FluidRestDensity);
		Fluid.rho_adv[i] = density_adv;
		Fluid.d_rho[i] = d_density;
		Fluid.avg_density_adv += density_adv;
		Fluid.avg_d_density += d_density;
	});
	Fluid.avg_density_adv /= Fluid.size;
	Fluid.avg_d_density /= Fluid.size;
}
void HinaPE::DFSPHSolverCPU::correct_density_error(real dt)
{
	compute_density_error(dt);
	int iter = 0;
	real max_error_d = 0.01f;
	while ((Fluid.avg_density_adv - FluidRestDensity) > max_error_d && iter < 100)
	{
		compute_kappa_density(dt);
		parallel_for(Fluid.size, [&](size_t i)
		{
			Vector sum{0, 0, 0};
			Vector x_i = Fluid.x[i];
			real m_i = Fluid.m[i];
			real rho_i = Fluid.rho[i];
			real kappa_i = Fluid.kappa_density[i];

			for_each_neighbor_fluid(
					i, [&](size_t j, Vector x_j)
					{
						real m_j = Fluid.m[j];
						real rho_j = Fluid.rho[j];
						real kappa_j = Fluid.kappa_density[j];
						Vector grad = m_j * (kappa_i / rho_i + kappa_j / rho_j) * Kernel.gradient(x_i - x_j);
						sum += grad;
					});

			for_each_neighbor_static_boundary(
					i, [&](size_t j, Vector x_j, size_t boundary_idx)
					{
						real m_j = StaticBoundaries[boundary_idx].m[j];
						real rho_j = StaticBoundaries[boundary_idx].rho[j];
						real kappa_j = 0;
						Vector grad = m_j * (kappa_i / rho_i + kappa_j / rho_j) * Kernel.gradient(x_i - x_j);
						sum += grad;
					});

			Fluid.v[i] -= dt * sum;
		});
		compute_density_error(dt);
		++iter;
	}
}
void HinaPE::DFSPHSolverCPU::correct_divergence_error(real dt)
{
	parallel_for(Fluid.size, [&](size_t i)
	{
		compute_density_error(dt);

		int iter = 0;
		fpreal max_error_v = 10.;
		while (Fluid.avg_d_density > max_error_v && iter < 100)
		{
			compute_kappa_divergence(dt);
			parallel_for(Fluid.size, [&](size_t i)
			{
				Vector sum{0, 0, 0};
				Vector x_i = Fluid.x[i];
				real m_i = Fluid.m[i];
				real rho_i = Fluid.rho[i];
				real kappa_i = Fluid.kappa_divergence[i];

				for_each_neighbor_fluid(
						i, [&](size_t j, Vector x_j)
						{
							real m_j = Fluid.m[j];
							real rho_j = Fluid.rho[j];
							real kappa_j = Fluid.kappa_divergence[j];
							Vector grad = m_j * (kappa_i / rho_i + kappa_j / rho_j) * Kernel.gradient(x_i - x_j);
							sum += grad;
						});

				for_each_neighbor_static_boundary(
						i, [&](size_t j, Vector x_j, size_t boundary_idx)
						{
							real m_j = StaticBoundaries[boundary_idx].m[j];
							real rho_j = StaticBoundaries[boundary_idx].rho[j];
							real kappa_j = 0;
							Vector grad = m_j * (kappa_i / rho_i + kappa_j / rho_j) * Kernel.gradient(x_i - x_j);
							sum += grad;
						});

				Fluid.v[i] -= dt * sum;
			});
		}
	});
}
