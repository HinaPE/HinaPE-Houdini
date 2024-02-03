#include "GAS_Hina_DFSPHSolver.h"
#include <DFSPH/SIM_Hina_DFSPHParticles.h>
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
