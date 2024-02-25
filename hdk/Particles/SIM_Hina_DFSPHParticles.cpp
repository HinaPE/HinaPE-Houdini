#include "SIM_Hina_DFSPHParticles.h"
#include <CUDA_HinaPE/kernels.h>

SIM_HINA_DERIVED_GEOMETRY_CLASS_IMPLEMENT(
		DFSPHParticles,
		Particles,
		true,
)
void SIM_Hina_DFSPHParticles::_init_DFSPHParticles()
{
	this->alpha_cache.clear();
	this->kappa_cache.clear();
	this->density_adv_cache.clear();
	this->d_density_cache.clear();
}
void SIM_Hina_DFSPHParticles::_makeEqual_DFSPHParticles(const SIM_Hina_DFSPHParticles *src)
{
	this->alpha_cache = src->alpha_cache;
	this->kappa_cache = src->kappa_cache;
	this->density_adv_cache = src->density_adv_cache;
	this->d_density_cache = src->d_density_cache;
}
void SIM_Hina_DFSPHParticles::_setup_gdp(GU_Detail *gdp) const
{
	SIM_Hina_Particles::_setup_gdp(gdp);
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_DFSPH_ALPHA, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_DFSPH_KAPPA, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_DFSPH_DENSITY_ADV, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_DFSPH_D_DENSITY, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
}
void SIM_Hina_DFSPHParticles::commit()
{
	SIM_Hina_Particles::commit();
	SIM_GeometryAutoWriteLock lock(this);
	GU_Detail &gdp = lock.getGdp();
	GA_RWHandleF alpha_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DFSPH_ALPHA);
	GA_RWHandleF kappa_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DFSPH_KAPPA);
	GA_RWHandleF density_adv_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DFSPH_DENSITY_ADV);
	GA_RWHandleF d_density_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DFSPH_D_DENSITY);
	GA_Offset pt_off;
	GA_FOR_ALL_PTOFF(&gdp, pt_off)
		{
			fpreal alpha = alpha_cache[pt_off];
			fpreal kappa = kappa_cache[pt_off];
			fpreal density_adv = density_adv_cache[pt_off];
			fpreal d_density = d_density_cache[pt_off];
			alpha_handle.set(pt_off, alpha);
			kappa_handle.set(pt_off, kappa);
			density_adv_handle.set(pt_off, density_adv);
			d_density_handle.set(pt_off, d_density);
		}
}
