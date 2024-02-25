#include "GAS_Hina_DFSPHSolver.h"
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
bool GAS_Hina_DFSPHSolver::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_Hina_DFSPHParticles *DFSPH_particles = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_DFSPHParticles);
	CHECK_NULL_RETURN_BOOL(DFSPH_particles)
	std::map<UT_String, SIM_Hina_Akinci2012BoundaryParticles *> akinci_boundaries = FetchAllAkinciBoundaries(obj);


	/// REFER TO PAPER [Divergence-Free Smoothed Particle Hydrodynamics], Section 3.1, Algorithm 1
	/// ========== 1. init neighborhoods ==========
	// perform neighbor search [done by `GAS_Hina_BuildNeighborLists`]

	/// ========== 2. compute densities and factors alpha ==========
	// compute densities [done by `GAS_Hina_UpdateDensityAkinci`]
	// compute alpha factors
	calculate_alpha(DFSPH_particles, akinci_boundaries);


	/// ========== 3. start simulation loop ==========
	// 3.1 apply CFL condition
	fpreal dt_CFL = calculate_CFL_dt(DFSPH_particles, timestep);

//	// 3.2 predict velocities v_i*
//	UT_Vector3 gravity = {0, -9.8, 0}; // TODO: enable more external forces
//	{
//		GA_FOR_ALL_PTOFF(gdp_fluid, pt_off)
//			{
//				UT_Vector3 v = velocity_handle.get(pt_off);
//				fpreal m = mass_handle.get(pt_off);
//				UT_Vector3 f = m * gravity;
//				UT_Vector3 a = f / m;
//				UT_Vector3 v_star = v + a * dt_CFL;
//				DFSPH_particles->velocity_cache[pt_off] = v_star;
//			}
//	}
//	// compute Drho/Dt
//	{
//		GA_FOR_ALL_PTOFF(gdp_fluid, pt_off)
//			{
//				fpreal Drho = 0; // self Drho is 0
//				int num_neighbors = 0;
//				UT_Vector3 x_i = gdp_fluid->getPos3(pt_off);
//				UT_Vector3 v_i = DFSPH_particles->velocity_cache[pt_off];
//				fpreal m_i = mass_handle.get(pt_off);
//				fpreal V_i = volume_handle.get(pt_off);
//				DFSPH_particles->for_each_neighbor_self(pt_off, [&](const GA_Offset &n_off, const UT_Vector3 &n_pos)
//				{
//					UT_Vector3 x_j = n_pos;
//					UT_Vector3 v_j = DFSPH_particles->velocity_cache[n_off];
//					fpreal m_j = mass_handle.get(n_off);
//					fpreal V_j = volume_handle.get(n_off);
//					Drho += m_j * (v_i - v_j).dot(kernel.gradient(x_i - x_j));
//					++num_neighbors;
//				});
//				for (const auto &pair: akinci_boundaries)
//				{
//					UT_String name = pair.first;
//					SIM_Hina_Akinci2012BoundaryParticles *akinci_boundary = pair.second;
//					SIM_GeometryAutoReadLock _(akinci_boundary);
//					const GU_Detail *gdp_boundary = _.getGdp();
//					GA_ROHandleV3 velocity_handle_boundary = gdp_boundary->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VELOCITY);
//					GA_ROHandleF volume_handle_boundary = gdp_boundary->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VOLUME);
//					GA_ROHandleF mass_handle_boundary = gdp_boundary->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_MASS);
//					akinci_boundary->for_each_neighbor_others(pt_off, [&](const GA_Offset &n_off, const UT_Vector3 &n_pos)
//					{
//						UT_Vector3 x_j = n_pos;
//						UT_Vector3 v_j = velocity_handle_boundary.get(n_off);
//						fpreal m_j = mass_handle_boundary.get(n_off);
//						fpreal V_j = volume_handle_boundary.get(n_off);
//						Drho += m_j * (v_i - v_j).dot(kernel.gradient(x_i - x_j));
//						++num_neighbors;
//					}, name);
//				}
//				Drho = std::max(Drho, 0.);
//				if (num_neighbors < 15)
//					Drho = 0;
//				DFSPH_particles->Drho_cache[pt_off] = Drho;
//				Drho_handle.set(pt_off, Drho);
//			}
//	}

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
