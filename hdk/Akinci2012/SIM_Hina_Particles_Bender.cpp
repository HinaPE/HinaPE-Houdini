#include "SIM_Hina_Particles_Bender.h"

SIM_HINA_DERIVED_GEOMETRY_CLASS_IMPLEMENT(
		Particles_Bender,
		Particles,
		true,
		HINA_FLOAT_PARAMETER(SolidDensity, 1000.) \
        HINA_BOOL_PARAMETER(IsDynamic, false) \
        TARGET_PARTICLE_GEOMETRY(SIM_Hina_Particles_Akinci)
)
void SIM_Hina_Particles_Bender::_init_Particles_Bender()
{
	this->_inited = false;
}
void SIM_Hina_Particles_Bender::_makeEqual_Particles_Bender(const SIM_Hina_Particles_Bender *src)
{
	this->_inited = src->_inited;
}
void SIM_Hina_Particles_Bender::_setup_gdp(GU_Detail *gdp) const
{
	SIM_Hina_Particles::_setup_gdp(gdp);
}
void SIM_Hina_Particles_Bender::load_sop(SIM_Object *boundary_obj)
{
	// TODO: load and init Volume Map
}
std::vector<SIM_Hina_Particles_Bender *> FetchAllBenderBoundaries(SIM_Object *fluid_obj)
{
	std::vector<SIM_Hina_Particles_Bender *> res;
	SIM_ObjectArray affectors;
	fluid_obj->getAffectors(affectors, "SIM_RelationshipCollide");
	exint num_affectors = affectors.entries();
	for (int i = 0; i < num_affectors; ++i)
	{
		SIM_Object *obj_collider = affectors(i);
		if (obj_collider->getName().equal(fluid_obj->getName()))
			continue;
		UT_String boundary_obj_name = obj_collider->getName();
		SIM_Hina_Particles_Bender *boundary_akinci = SIM_DATA_GET(*obj_collider, SIM_Hina_Particles_Bender::DATANAME, SIM_Hina_Particles_Bender);
		if (boundary_akinci)
		{
			if (!boundary_akinci->_inited)
				boundary_akinci->load_sop(obj_collider);
			res.emplace_back(boundary_akinci);
		}
	}
	return res;
}
