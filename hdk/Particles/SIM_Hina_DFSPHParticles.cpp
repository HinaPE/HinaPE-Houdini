#include "SIM_Hina_DFSPHParticles.h"
#include <CUDA_HinaPE/kernels.h>

SIM_HINA_DERIVED_GEOMETRY_CLASS_IMPLEMENT(
		DFSPHParticles,
		Particles,
		true,
)
void SIM_Hina_DFSPHParticles::_init_DFSPHParticles() {}
void SIM_Hina_DFSPHParticles::_makeEqual_DFSPHParticles(const SIM_Hina_DFSPHParticles *src) {}
void SIM_Hina_DFSPHParticles::_setup_gdp(GU_Detail *gdp) const
{
	SIM_Hina_Particles::_setup_gdp(gdp);
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_DFSPH_ALPHA, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
}
void SIM_Hina_DFSPHParticles::calculate_alpha()
{
	SIM_GeometryAutoWriteLock lock(this);
	GU_Detail &gdp = lock.getGdp();
	GA_ROHandleF mass_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_MASS);
	GA_ROHandleF volume_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VOLUME);

	HinaPE::CubicSplineKernel<false> kernel(getTargetSpacing() * getKernelRadiusOverTargetSpacing());

	GA_Offset pt_off;
	GA_FOR_ALL_PTOFF(&gdp, pt_off)
		{
			fpreal sum_grad;
			UT_Vector3D grad{0., 0., 0.};
			fpreal mass = mass_handle.get(pt_off);
			fpreal volume = volume_handle.get(pt_off);
			alpha_cache[pt_off] = 0;

			UT_Vector3D p_i = gdp.getPos3(pt_off);
			for_each_neighbor_self(pt_off, [&](const GA_Offset &n_off, const UT_Vector3 &n_pos)
			{
				UT_Vector3D p_j = n_pos;
				const UT_Vector3D r = p_i - p_j;
				grad += kernel.derivative(r.length());
			});
		}
}
