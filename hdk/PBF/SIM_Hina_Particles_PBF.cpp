#include "SIM_Hina_Particles_PBF.h"

SIM_HINA_DERIVED_GEOMETRY_CLASS_IMPLEMENT(
		Particles_PBF,
		Particles,
		true,
)
void SIM_Hina_Particles_PBF::_init_Particles_PBF()
{
	this->p_x = nullptr;
}
void SIM_Hina_Particles_PBF::_makeEqual_Particles_PBF(const SIM_Hina_Particles_PBF *src)
{
	this->p_x = src->p_x;
}
void SIM_Hina_Particles_PBF::_setup_gdp(GU_Detail *gdp) const
{
	SIM_Hina_Particles::_setup_gdp(gdp);
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_PBF, HINA_GEOMETRY_ATTRIBUTE_TYPE_VECTOR3)
}
void SIM_Hina_Particles_PBF::commit()
{
	SIM_Hina_Particles::commit();

	if (p_x == nullptr)
	{
		std::cout << "SIM_Hina_Particles_PBF::load() called with nullptr" << std::endl;
		return;
	}

	size_t size = p_x->size();
	if (size == 0)
		return;

	SIM_GeometryAutoWriteLock lock(this);
	GU_Detail &gdp = lock.getGdp();
	GA_RWHandleV3 p_x_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_PBF);
	GA_Offset pt_off;
	GA_FOR_ALL_PTOFF(&gdp, pt_off)
		{
			UT_Vector3 px = (*p_x)[offset2index[pt_off]];
			p_x_handle.set(pt_off, px);
		}
}
