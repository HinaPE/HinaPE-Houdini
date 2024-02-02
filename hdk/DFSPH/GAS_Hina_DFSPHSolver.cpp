#include "GAS_Hina_DFSPHSolver.h"
#include <DFSPH/SIM_Hina_DFSPHParticles.h>
#include <array>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		DFSPHSolver,
		true,
		false,
		TARGET_PARTICLE_GEOMETRY(SIM_Hina_DFSPHParticles)
)

template<size_t accuracy = 10000>
struct CubicSplineKernel
{
	fpreal kernel(const fpreal r)
	{
		int cache_idx = std::floor(r * 10000);
		return kernel_cache[cache_idx];
	}

	fpreal derivative(const fpreal r)
	{
		int cache_idx = std::floor(r * 10000);
		return derivative_cache[cache_idx];
	}

	fpreal h;
	std::array<fpreal, accuracy> kernel_cache;
	std::array<fpreal, accuracy> derivative_cache;
	constexpr CubicSplineKernel(fpreal h) : h(h), kernel_cache(), derivative_cache()
	{
		for (int i = 0; i < accuracy; ++i)
		{
			fpreal r = i / (fpreal) accuracy;
			fpreal q = 1 - r;
			if (r < 0.5)
			{
				kernel_cache[i] = 2.0 / 3.0 - 4.0 * r * r + 4.0 * r * r * r;
				derivative_cache[i] = -8.0 * r + 12.0 * r * r;
			} else
			{
				kernel_cache[i] = 4.0 / 3.0 - 4.0 * r + 4.0 * r * r - 4.0 / 3.0 * r * r * r;
				derivative_cache[i] = -4.0 + 8.0 * r - 4.0 * r * r;
			}
		}
	}
};

void GAS_Hina_DFSPHSolver::_init() {}
void GAS_Hina_DFSPHSolver::_makeEqual(const GAS_Hina_DFSPHSolver *src) {}
bool GAS_Hina_DFSPHSolver::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_Hina_DFSPHParticles *fluid_particles = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_DFSPHParticles);
	CHECK_NULL_RETURN_BOOL(fluid_particles)
	fpreal kernel_radius = fluid_particles->getTargetSpacing() * fluid_particles->getKernelRadiusOverTargetSpacing();
	CubicSplineKernel kernel(kernel_radius);

	SIM_GeometryAutoWriteLock lock(fluid_particles);
	GU_Detail &gdp = lock.getGdp();

	GA_Offset pt_off;
	GA_FOR_ALL_PTOFF(&gdp, pt_off)
		{
			const std::vector<GA_Offset> &neighbors = fluid_particles->neighbor_lists_cache[pt_off];

			UT_Vector3 pos_i = gdp.getPos3(pt_off);
			for (const GA_Offset &n_off: neighbors)
			{
				UT_Vector3 pos_j = gdp.getPos3(n_off);
			}
		}

	return true;
}
bool GAS_Hina_DFSPHSolver::_compute_alpha()
{

	return true;
}
