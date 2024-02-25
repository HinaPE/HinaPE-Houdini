#include "GAS_Hina_DFSPHSolver.h"
#include <Density/GAS_Hina_UpdateDensityAkinci.h>
#include <Neighbor/GAS_Hina_BuildNeighborLists.h>
#include <Particles/SIM_Hina_DFSPHParticles.h>
#include <Particles/SIM_Hina_Akinci2012BoundaryParticles.h>
#include <CUDA_HinaPE/kernels.h>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		DFSPHSolver,
		true,
		false,
		TARGET_PARTICLE_GEOMETRY(SIM_Hina_DFSPHParticles)
)

void GAS_Hina_DFSPHSolver::_init() {}
void GAS_Hina_DFSPHSolver::_makeEqual(const GAS_Hina_DFSPHSolver *src) {}
bool GAS_Hina_DFSPHSolver::_solve(SIM_Engine &engine, SIM_Object *fluid_obj, SIM_Time time, SIM_Time timestep)
{
	SIM_Hina_DFSPHParticles *DFSPH_particles = SIM_DATA_CAST(getGeometryCopy(fluid_obj, GAS_NAME_GEOMETRY), SIM_Hina_DFSPHParticles);
	CHECK_NULL_RETURN_BOOL(DFSPH_particles)
	std::map<UT_String, SIM_Hina_Akinci2012BoundaryParticles *> akinci_boundaries = FetchAllAkinciBoundaries(fluid_obj);


	/// REFER TO PAPER [Divergence-Free Smoothed Particle Hydrodynamics], Section 3.1, Algorithm 1
	fpreal remain_time = timestep;
	while (remain_time > 1e-6)
	{
		fpreal dt_CFL = calculate_CFL_dt(DFSPH_particles, remain_time);
		calculate_non_pressure_force(DFSPH_particles);
		DFSPH_particles->advect_velocity(dt_CFL);
		correct_density_error(DFSPH_particles, akinci_boundaries);
		DFSPH_particles->advect_position(dt_CFL);
		DFSPH_particles->neighbor_lists_builder->update_search_engine(DFSPH_particles, akinci_boundaries);
		GAS_Hina_UpdateDensityAkinci::calculate_density(DFSPH_particles, akinci_boundaries);
		calculate_alpha(DFSPH_particles, akinci_boundaries);
		correct_divergence_error(DFSPH_particles, akinci_boundaries);
		remain_time -= dt_CFL;
	}
	return true;
}
fpreal GAS_Hina_DFSPHSolver::calculate_CFL_dt(SIM_Hina_DFSPHParticles *fluid, fpreal t_max)
{
	fpreal kernel_radius = fluid->getTargetSpacing() * fluid->getKernelRadiusOverTargetSpacing();
	fpreal dt_CFL = 0.25 * kernel_radius;
	fpreal max_vel = 1e-6;
	fluid->for_each_offset(
			[&](const GA_Offset &pt_off)
			{
				UT_Vector3 v = fluid->velocity_cache[pt_off];
				fpreal v_len = v.length();
				if (v_len > max_vel)
					max_vel = v_len;
			}
	);
	dt_CFL /= max_vel;
	dt_CFL = std::min(dt_CFL, t_max);
	return dt_CFL;
}
void GAS_Hina_DFSPHSolver::calculate_non_pressure_force(SIM_Hina_DFSPHParticles *fluid)
{
	fluid->clear_force();
	fluid->calculate_force_gravity();
}
void GAS_Hina_DFSPHSolver::calculate_alpha(SIM_Hina_DFSPHParticles *fluid, std::map<UT_String, SIM_Hina_Akinci2012BoundaryParticles *> &akinci_boundaries)
{
	fpreal kernel_radius = fluid->getTargetSpacing() * fluid->getKernelRadiusOverTargetSpacing();
	HinaPE::CubicSplineKernel<false> kernel(kernel_radius);

	fluid->for_each_offset(
			[&](const GA_Offset &pt_off)
			{
				fpreal alpha = 0;
				fpreal grad_square_sum = 0;
				fpreal grad_sum_square = 0;
				UT_Vector3 grad_sum = {0, 0, 0};

				UT_Vector3 x_i = fluid->position_cache[pt_off];
				fpreal m_i = fluid->mass_cache[pt_off];
				fpreal rho_i = fluid->density_cache[pt_off];
				// self contribution
				{
					UT_Vector3 grad = kernel.gradient(UT_Vector3(0., 0., 0.)); // TODO: maybe error here
					grad_sum += grad;
					grad_square_sum += grad.length2();
				}
				// fluid neighbors contribution
				fluid->for_each_neighbor_self(
						pt_off, [&](const GA_Offset &n_off, const UT_Vector3 &n_pos)
						{
							UT_Vector3 x_j = n_pos;
							fpreal m_j = fluid->mass_cache[n_off];
							const UT_Vector3 r = x_i - x_j;
							UT_Vector3 grad = m_j * kernel.gradient(r);
							grad_sum += grad;
							grad_square_sum += grad.length2();
						}
				);
				// akinci boundaries neighbors contribution
				for (const auto &pair: akinci_boundaries)
				{
					fluid->for_each_neighbor_others(pt_off, [&](const GA_Offset &n_off, const UT_Vector3 &n_pos)
					{
						UT_Vector3 x_j = n_pos;
						fpreal m_j = pair.second->mass_cache[n_off];
						const UT_Vector3 r = x_i - x_j;
						UT_Vector3 grad = m_j * kernel.gradient(r);
						grad_sum += grad;
						grad_square_sum += grad.length2();
					}, pair.first);
				}
				grad_sum_square = grad_sum.length2();
				fpreal denominator = grad_square_sum + grad_sum_square;
				if (denominator < 1e-6)
					denominator = 1e-6;
				alpha = rho_i / denominator;
				fluid->alpha_cache[pt_off] = alpha;
			}
	);
}
void GAS_Hina_DFSPHSolver::correct_divergence_error(SIM_Hina_DFSPHParticles *fluid, std::map<UT_String, SIM_Hina_Akinci2012BoundaryParticles *> &akinci_boundary)
{

}
void GAS_Hina_DFSPHSolver::correct_density_error(SIM_Hina_DFSPHParticles *fluid, std::map<UT_String, SIM_Hina_Akinci2012BoundaryParticles *> &akinci_boundary)
{

}
