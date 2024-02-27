#include "SIM_Hina_DFSPH_Particles.h"

SIM_HINA_DERIVED_GEOMETRY_CLASS_IMPLEMENT(
		DFSPH_Particles,
		Particles,
		true,
		TARGET_PARTICLE_GEOMETRY(SIM_Hina_DFSPH_Particles)
)
void SIM_Hina_DFSPH_Particles::_init_DFSPH_Particles()
{
	this->alpha = nullptr;
	this->kappa_density = nullptr;
	this->kappa_divergence = nullptr;
	this->rho_adv = nullptr;
	this->d_rho = nullptr;
}
void SIM_Hina_DFSPH_Particles::_makeEqual_DFSPH_Particles(const SIM_Hina_DFSPH_Particles *src)
{
	this->alpha = src->alpha;
	this->kappa_density = src->kappa_density;
	this->kappa_divergence = src->kappa_divergence;
	this->rho_adv = src->rho_adv;
	this->d_rho = src->d_rho;
}
void SIM_Hina_DFSPH_Particles::_setup_gdp(GU_Detail *gdp) const
{
	SIM_Hina_Particles::_setup_gdp(gdp);
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_DFSPH_ALPHA, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_DFSPH_KAPPA_DENSITY, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_DFSPH_KAPPA_DIVERGENCE, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_DFSPH_DENSITY_ADV, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_DFSPH_D_DENSITY, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
}
void SIM_Hina_DFSPH_Particles::commit()
{
	SIM_Hina_Particles::commit();
	SIM_GeometryAutoWriteLock lock(this);
	GU_Detail &gdp = lock.getGdp();
	GA_RWHandleF alpha_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DFSPH_ALPHA);
	GA_RWHandleF kappa_density_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DFSPH_KAPPA_DENSITY);
	GA_RWHandleF kappa_divergence_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DFSPH_KAPPA_DIVERGENCE);
	GA_RWHandleF density_adv_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DFSPH_DENSITY_ADV);
	GA_RWHandleF d_density_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DFSPH_D_DENSITY);

	GA_Offset pt_off;
	GA_FOR_ALL_PTOFF(&gdp, pt_off)
		{
			fpreal a = (*alpha)[offset2index[pt_off]];
			fpreal k_d = (*kappa_density)[offset2index[pt_off]];
			fpreal k_div = (*kappa_divergence)[offset2index[pt_off]];
			fpreal d_adv = (*rho_adv)[offset2index[pt_off]];
			fpreal d_d = (*d_rho)[offset2index[pt_off]];
			alpha_handle.set(pt_off, a);
			kappa_density_handle.set(pt_off, k_d);
			kappa_divergence_handle.set(pt_off, k_div);
			density_adv_handle.set(pt_off, d_adv);
			d_density_handle.set(pt_off, d_d);
		}
}
