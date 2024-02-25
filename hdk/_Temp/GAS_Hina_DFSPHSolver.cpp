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
	fpreal kernel_radius = DFSPH_particles->getTargetSpacing() * DFSPH_particles->getKernelRadiusOverTargetSpacing();
	HinaPE::CubicSplineKernel<false> kernel(kernel_radius);

//	SIM_GeometryAutoWriteLock lock(DFSPH_particles);
//	GU_Detail *gdp_fluid = &lock.getGdp();
//	GA_Offset pt_off;
//	GA_ROHandleV3 position_handle = gdp_fluid->getP();
//	GA_ROHandleV3 velocity_handle = gdp_fluid->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VELOCITY);
//	GA_ROHandleV3 force_handle = gdp_fluid->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_FORCE);
//	GA_ROHandleF volume_handle = gdp_fluid->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VOLUME);
//	GA_ROHandleF mass_handle = gdp_fluid->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_MASS);
//	GA_ROHandleF density_handle = gdp_fluid->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DENSITY);
//	GA_RWHandleF alpha_handle = gdp_fluid->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DFSPH_ALPHA);
//	GA_RWHandleF Drho_handle = gdp_fluid->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DFSPH_DRHO);
//
//
//
//
//
//	/// REFER TO PAPER [Divergence-Free Smoothed Particle Hydrodynamics], Section 3.1, Algorithm 1
//	/// ========== 1. init neighborhoods ==========
//	// perform neighbor search [done by `GAS_Hina_BuildNeighborLists`]
//
//	/// ========== 2. compute densities and factors alpha ==========
//	// compute densities [done by `GAS_Hina_UpdateDensityAkinci`]
//	// compute alpha factors
//	{
//		GA_FOR_ALL_PTOFF(gdp_fluid, pt_off)
//			{
//				fpreal alpha = 0;
//				fpreal grad_square_sum = 0;
//				fpreal grad_sum_square = 0;
//				UT_Vector3 grad_sum = {0, 0, 0};
//
//				UT_Vector3 x_i = DFSPH_particles->position_cache[pt_off];
//				fpreal m_i = DFSPH_particles->mass_cache[pt_off];
//				fpreal rho_i = DFSPH_particles->density_cache[pt_off];
//				// self contribution
//				{
//					UT_Vector3 grad = kernel.gradient(UT_Vector3(0., 0., 0.)); // TODO: maybe error here
//					grad_sum += grad;
//					grad_square_sum += grad.length2();
//				}
//				// fluid neighbors contribution
//				DFSPH_particles->for_each_neighbor_self(pt_off, [&](const GA_Offset &n_off, const UT_Vector3 &n_pos)
//				{
//					UT_Vector3 x_j = n_pos;
//					fpreal m_j = mass_handle.get(n_off);
//					const UT_Vector3 r = x_i - x_j;
//					UT_Vector3 grad = m_j * kernel.gradient(r);
//					grad_sum += grad;
//					grad_square_sum += grad.length2();
//				});
//				// akinci boundaries neighbors contribution
//				for (const auto &pair: akinci_boundaries)
//				{
//					SIM_GeometryAutoReadLock _(pair.second);
//					const GU_Detail *gdp_boundary = _.getGdp();
//					GA_ROHandleF mass_handle_boundary = gdp_boundary->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_MASS);
//					DFSPH_particles->for_each_neighbor_others(pt_off, [&](const GA_Offset &n_off, const UT_Vector3 &n_pos)
//					{
//						UT_Vector3 x_j = n_pos;
//						fpreal m_j = mass_handle_boundary.get(n_off);
//						const UT_Vector3 r = x_i - x_j;
//						UT_Vector3 grad = m_j * kernel.gradient(r);
//						grad_sum += grad;
//						grad_square_sum += grad.length2();
//					}, pair.first);
//				}
//				grad_sum_square = grad_sum.length2();
//				fpreal denominator = grad_square_sum + grad_sum_square;
//				if (denominator < 1e-6)
//					denominator = 1e-6;
//				alpha = rho_i / denominator;
//				DFSPH_particles->alpha_cache[pt_off] = alpha;
//				alpha_handle.set(pt_off, alpha);
//			}
//	}
//
//
//	/// ========== 3. start simulation loop ==========
//	// 3.1 apply CFL condition
//	fpreal t_max = timestep;
//	fpreal dt_CFL = 0.25 * kernel_radius;
//	{
//		fpreal max_vel = 1e-6;
//		GA_FOR_ALL_PTOFF(gdp_fluid, pt_off)
//			{
//				UT_Vector3 v = velocity_handle.get(pt_off);
//				fpreal v_len = v.length();
//				if (v_len > max_vel)
//					max_vel = v_len;
//			}
//		dt_CFL /= max_vel;
//		dt_CFL = std::min(dt_CFL, t_max);
//	}
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
