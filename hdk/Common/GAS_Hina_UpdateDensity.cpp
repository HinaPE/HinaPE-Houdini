#include "GAS_Hina_UpdateDensity.h"
#include <Common/SIM_Hina_Particles.h>
#include <CUDA_CubbyFlow/Core/Particle/SPHKernels.hpp>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		UpdateDensity,
		true,
		false,
		TARGET_PARTICLE_GEOMETRY(SIM_Hina_Particles)
				HINA_FLOAT_PARAMETER(KernelRadius, 0.036)
)

void GAS_Hina_UpdateDensity::_init() {}
void GAS_Hina_UpdateDensity::_makeEqual(const GAS_Hina_UpdateDensity *src) {}
bool GAS_Hina_UpdateDensity::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_Hina_Particles *particles = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_Particles);
	CHECK_NULL_RETURN_BOOL(particles)
	fpreal kernel_radius = particles->getTargetSpacing() * particles->getKernelRadiusOverTargetSpacing();

	SIM_GeometryAutoWriteLock lock(particles);
	GU_Detail &gdp = lock.getGdp();
	GA_ROHandleV3 pos_handle = gdp.getP();
	GA_RWHandleF mass_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_MASS);
	GA_RWHandleF density_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DENSITY);

	CubbyFlow::SPHStdKernel3 kernel(kernel_radius);
	for (auto &n_list: particles->neighbor_lists_cache)
	{
		double sum = 0.;

		GA_Offset pt_off = n_list.first;
		UT_Vector3 pt_pos = gdp.getPos3(pt_off);
		for (auto &n_off: n_list.second)
		{
			UT_Vector3 n_pos = pos_handle.get(n_off);
			fpreal dist = pt_pos.distance(n_pos);
			sum += kernel(dist);
		}

		fpreal pt_mass = mass_handle.get(pt_off);
		density_handle.set(pt_off, pt_mass * sum);
	}
	return true;
}
