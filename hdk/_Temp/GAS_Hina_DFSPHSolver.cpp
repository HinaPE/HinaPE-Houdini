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
	HinaPE::CubicSplineKernel<false> kernel(DFSPH_particles->getTargetSpacing() * DFSPH_particles->getKernelRadiusOverTargetSpacing());

	SIM_GeometryAutoReadLock lock(DFSPH_particles);
	const GU_Detail *gdp_fluid = lock.getGdp();
	GA_ROHandleF volume_handle = gdp_fluid->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VOLUME);
	GA_ROHandleF mass_handle = gdp_fluid->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_MASS);
	GA_ROHandleF density_handle = gdp_fluid->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DENSITY);






	/// REFER TO PAPER [Divergence-Free Smoothed Particle Hydrodynamics], Section 3.1, Algorithm 1
	/// ========== 1. init neighborhoods ==========
	// perform neighbor search [done by `GAS_Hina_BuildNeighborLists`]

	/// ========== 2. compute densities and factors alpha ==========
	// compute densities [done by `GAS_Hina_UpdateDensityAkinci`]
	// compute alpha factors
	GA_Offset pt_off;
	GA_FOR_ALL_PTOFF(gdp_fluid, pt_off)
		{
			fpreal alpha = 0;
			fpreal grad_square_sum = 0;
			fpreal grad_sum_square = 0;
			UT_Vector3 grad_sum = {0, 0, 0};

			UT_Vector3 x_i = gdp_fluid->getPos3(pt_off);
			fpreal m_i = mass_handle.get(pt_off);
			fpreal rho_i = density_handle.get(pt_off);
			// self contribution
			{
				UT_Vector3 grad = kernel.gradient(UT_Vector3(0., 0., 0.));
				grad_sum += grad;
				grad_square_sum += grad.length2();
			}
			// fluid neighbors contribution
			DFSPH_particles->for_each_neighbor_self(pt_off, [&](const GA_Offset &n_off, const UT_Vector3 &n_pos)
			{
				UT_Vector3 x_j = n_pos;
				fpreal m_j = mass_handle.get(n_off);
				const UT_Vector3 r = x_i - x_j;
				UT_Vector3 grad = m_j * kernel.gradient(r);
				grad_sum += grad;
				grad_square_sum += grad.length2();
			});
			// akinci boundaries neighbors contribution
			DFSPH_particles->for_each_neighbor_others(pt_off, [&](const GA_Offset &n_off, const UT_Vector3 &n_pos)
			{
				UT_Vector3 x_j = n_pos;
				fpreal volume = volume_handle.get(n_off);
				const UT_Vector3 r = x_i - x_j;
				UT_Vector3 grad = 1000. * volume * kernel.gradient(r);
				grad_sum += grad;
				grad_sum_square += grad.length2();
			});
			grad_sum_square = grad_sum.length2();
			alpha = rho_i / (grad_square_sum + grad_sum_square);
			if (alpha < 1e-6)
				alpha = 1e-6;
		}
	return true;
}
