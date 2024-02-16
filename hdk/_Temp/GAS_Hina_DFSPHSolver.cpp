#include "GAS_Hina_DFSPHSolver.h"
#include <Particles/SIM_Hina_DFSPHParticles.h>
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
	HinaPE::CubicSplineKernel kernel(kernel_radius);

	SIM_GeometryAutoWriteLock lock(fluid_particles);
	GU_Detail &gdp = lock.getGdp();

	GA_Offset pt_off;
	GA_FOR_ALL_PTOFF(&gdp, pt_off)
		{
			fpreal alpha = 0;
			fpreal kappa = 0;
			fluid_particles->for_all_neighbors(pt_off, [&](const GA_Offset &neighbor)
			{
				const UT_Vector3 r = gdp.getPos3(neighbor) - gdp.getPos3(pt_off);
				const fpreal r_l = r.length();
				alpha += kernel.kernel(r_l);
				kappa += kernel.derivative(r_l);
			});
			fluid_particles->alpha_cache[pt_off] = alpha;
			fluid_particles->kappa_cache[pt_off] = kappa;
		}

	return true;
}
bool GAS_Hina_DFSPHSolver::_compute_alpha()
{
	return true;
}
