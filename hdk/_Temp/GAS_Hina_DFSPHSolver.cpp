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
	// ========== 1. Fetch Fluid Particles ==========
	SIM_Hina_DFSPHParticles *fluid_particles = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_DFSPHParticles);
	CHECK_NULL_RETURN_BOOL(fluid_particles)
	fpreal KernelRadius = fluid_particles->getTargetSpacing() * fluid_particles->getKernelRadiusOverTargetSpacing();
	HinaPE::CubicSplineKernel<false> kernel(KernelRadius);

	// ========== 2. Fetch Boundaries Particles ==========
	std::map<UT_String, SIM_Hina_Akinci2012BoundaryParticles *> boundaries_akinci;
	{
		SIM_ObjectArray affectors;
		obj->getAffectors(affectors, "SIM_RelationshipCollide");
		exint num_affectors = affectors.entries();
		for (int i = 0; i < num_affectors; ++i)
		{
			SIM_Object *obj_collider = affectors(i);
			if (obj_collider->getName().equal(obj->getName()))
				continue;

			UT_String boundary_obj_name = obj_collider->getName();
			SIM_Hina_Akinci2012BoundaryParticles *boundary_akinci = SIM_DATA_GET(*obj_collider, SIM_Hina_Akinci2012BoundaryParticles::DATANAME, SIM_Hina_Akinci2012BoundaryParticles);
			if (boundary_akinci)
				boundaries_akinci[boundary_obj_name] = boundary_akinci;
		}
	}


	// ========== 3. Compute Alpha ==========
	SIM_GeometryAutoReadLock lock_fluid(fluid_particles);
	const GU_Detail *gdp_fluid = lock_fluid.getGdp();

	GA_Offset pt_off;
	GA_FOR_ALL_PTOFF(gdp_fluid, pt_off)
		{
			fpreal alpha = 0;
			fpreal kappa = 0;
			fluid_particles->for_each_neighbor_fluid(pt_off, [&](const GA_Offset &n_off)
			{
				UT_Vector3 p_i = gdp_fluid->getPos3(pt_off);
				UT_Vector3 p_j = gdp_fluid->getPos3(n_off);
				const UT_Vector3 r = p_i - p_j;

//				const UT_Vector3 grad_p_j = gdp_fluid.getPos3(n_off);
//				const fpreal r_l = r.length();
//				alpha += kernel.kernel(r_l);
//				kappa += kernel.derivative(r_l);
			});

			for (const auto &pair: boundaries_akinci)
			{
				SIM_GeometryAutoReadLock lock_boundary(fluid_particles);
				const GU_Detail *gdp_boundary = lock_boundary.getGdp();

				UT_String boundary_name = pair.first;
				fluid_particles->for_each_neighbor_boundary(pt_off, [&](const GA_Offset &n_off)
				{
					UT_Vector3 p_i = gdp_fluid->getPos3(pt_off);
					UT_Vector3 p_j = gdp_boundary->getPos3(n_off);
					const UT_Vector3 r = p_i - p_j;
				}, boundary_name);
			}

			fluid_particles->alpha_cache[pt_off] = alpha;
			fluid_particles->kappa_cache[pt_off] = kappa;
		}
	return true;
}
