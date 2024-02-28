#include "SIM_Hina_Particles_DFSPH.h"

SIM_HINA_DERIVED_GEOMETRY_CLASS_IMPLEMENT(
		Particles_DFSPH,
		Particles,
		true,
		TARGET_PARTICLE_GEOMETRY(SIM_Hina_Particles_DFSPH)
)
SIM_Hina_Particles_DFSPH::~SIM_Hina_Particles_DFSPH()
{
	this->factor = nullptr;
	this->density_adv = nullptr;
}
void SIM_Hina_Particles_DFSPH::_init_Particles_DFSPH()
{
	this->factor = nullptr;
	this->density_adv = nullptr;
}
void SIM_Hina_Particles_DFSPH::_makeEqual_Particles_DFSPH(const SIM_Hina_Particles_DFSPH *src)
{
	this->factor = src->factor;
	this->density_adv = src->density_adv;
}
void SIM_Hina_Particles_DFSPH::_setup_gdp(GU_Detail *gdp) const
{
	SIM_Hina_Particles::_setup_gdp(gdp);
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_DFSPH_FACTOR, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_DFSPH_DENSITY_ADV, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
}
void SIM_Hina_Particles_DFSPH::commit()
{
	SIM_Hina_Particles::commit();

	if (factor == nullptr || density_adv == nullptr)
	{
		std::cout << "SIM_Hina_Particles_DFSPH::load() called with nullptr" << std::endl;
		return;
	}

	size_t size = factor->size();
	if (size == 0)
		return;

	SIM_GeometryAutoWriteLock lock(this);
	GU_Detail &gdp = lock.getGdp();
	GA_RWHandleF factor_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DFSPH_FACTOR);
	GA_RWHandleF density_adv_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DFSPH_DENSITY_ADV);
	GA_Offset pt_off;
	GA_FOR_ALL_PTOFF(&gdp, pt_off)
		{
			fpreal fc = (*factor)[offset2index[pt_off]];
			fpreal da = (*density_adv)[offset2index[pt_off]];
			factor_handle.set(pt_off, fc);
			density_adv_handle.set(pt_off, da);
		}
}
