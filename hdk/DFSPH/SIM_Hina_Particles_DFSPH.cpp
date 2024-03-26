#include "SIM_Hina_Particles_DFSPH.h"

SIM_HINA_DERIVED_GEOMETRY_CLASS_IMPLEMENT(
		Particles_DFSPH,
		Particles,
		true,
)
void SIM_Hina_Particles_DFSPH::_init_Particles_DFSPH()
{
	this->factor = nullptr;
	this->k = nullptr;
	this->density_adv = nullptr;
    this->BFLP = nullptr;
    this->VP = nullptr;
}
void SIM_Hina_Particles_DFSPH::_makeEqual_Particles_DFSPH(const SIM_Hina_Particles_DFSPH *src)
{
	this->factor = src->factor;
	this->k = src->k;
	this->density_adv = src->density_adv;
    this->BFLP = src->BFLP;
    this->VP = src->VP;
}
void SIM_Hina_Particles_DFSPH::_setup_gdp(GU_Detail *gdp) const
{
	SIM_Hina_Particles::_setup_gdp(gdp);
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_DFSPH_FACTOR, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_DFSPH_KAPPA_DENSITY, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_DFSPH_DENSITY_ADV, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
    HINA_GEOMETRY_POINT_ATTRIBUTE("BFLP", HINA_GEOMETRY_ATTRIBUTE_TYPE_INT)
    HINA_GEOMETRY_POINT_ATTRIBUTE("VP", HINA_GEOMETRY_ATTRIBUTE_TYPE_INT)
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
	GA_RWHandleF k_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DFSPH_KAPPA_DENSITY);
	GA_RWHandleF density_adv_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DFSPH_DENSITY_ADV);
    GA_RWHandleI BFLP_handle = gdp.findPointAttribute("BFLP");
    GA_RWHandleI VP_handle = gdp.findPointAttribute("VP");
	GA_Offset pt_off;
	GA_FOR_ALL_PTOFF(&gdp, pt_off)
		{
			fpreal fc = (*factor)[offset2index[pt_off]];
			fpreal kappa = (*k)[offset2index[pt_off]];
			fpreal da = (*density_adv)[offset2index[pt_off]];
            int bflp = (*BFLP)[offset2index[pt_off]];
            int vp = (*VP)[offset2index[pt_off]];
            BFLP_handle.set(pt_off, bflp);
            VP_handle.set(pt_off, vp);
			factor_handle.set(pt_off, fc);
			k_handle.set(pt_off, kappa);
			density_adv_handle.set(pt_off, da);
		}
}
