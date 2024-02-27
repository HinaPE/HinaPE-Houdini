#include "SIM_Hina_Akinci_Particles.h"

SIM_HINA_DERIVED_GEOMETRY_CLASS_IMPLEMENT(
		Akinci_Particles,
		Particles,
		true,
		HINA_FLOAT_PARAMETER(SolidDensity, 1000.) \
		TARGET_PARTICLE_GEOMETRY(SIM_Hina_Akinci_Particles)
)
void SIM_Hina_Akinci_Particles::_init_Akinci_Particles()
{
	this->_inited = false;
	this->_dynamic = false;
	this->offset_map.clear();
}
void SIM_Hina_Akinci_Particles::_makeEqual_Akinci_Particles(const SIM_Hina_Akinci_Particles *src)
{
	this->_inited = src->_inited;
	this->_dynamic = src->_dynamic;
	this->offset_map = src->offset_map;
}
void SIM_Hina_Akinci_Particles::_setup_gdp(GU_Detail *gdp) const
{
	SIM_Hina_Particles::_setup_gdp(gdp);
}
void SIM_Hina_Akinci_Particles::load_sop(SIM_Object *boundary_obj)
{
	// Reload the whole boundary particles
	if (!_inited)
	{
		std::map<GA_Offset, UT_Vector3> positions;
		{
			SIM_Geometry *boundary_sop = SIM_DATA_GET(*boundary_obj, SIM_GEOMETRY_DATANAME, SIM_Geometry);
			if (!boundary_sop)
				return;
			SIM_GeometryAutoReadLock lock(boundary_sop);
			const GU_Detail *gdp = lock.getGdp();
			GA_Offset pt_off;
			GA_FOR_ALL_PTOFF(gdp, pt_off)
				{
					UT_Vector3 pos = gdp->getPos3(pt_off);
					positions[pt_off] = pos;
				}
		}

		{
			SIM_GeometryAutoWriteLock lock(this);
			GU_Detail &gdp = lock.getGdp();
			GA_RWHandleF density_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DENSITY);
			fpreal rest_density = this->getSolidDensity();
			GA_Offset pt_off;
			for (auto &pair: positions)
			{
				pt_off = gdp.appendPoint();
				this->offset_map[pt_off] = pair.first;
				gdp.setPos3(pt_off, pair.second);
				density_handle.set(pt_off, rest_density);
			}
		}
		_inited = true;
		return;
	}

	// update the boundary particles
	if (_dynamic)
	{
		std::map<GA_Offset, UT_Vector3> positions;
		{
			SIM_Geometry *boundary_sop = SIM_DATA_GET(*boundary_obj, SIM_GEOMETRY_DATANAME, SIM_Geometry);
			if (!boundary_sop)
				return;
			SIM_GeometryAutoReadLock lock(boundary_sop);
			const GU_Detail *gdp = lock.getGdp();
			GA_Offset pt_off;
			GA_FOR_ALL_PTOFF(gdp, pt_off)
				{
					UT_Vector3 pos = gdp->getPos3(pt_off);
					positions[pt_off] = pos;
				}
		}

		{
			SIM_GeometryAutoWriteLock lock(this);
			GU_Detail &gdp = lock.getGdp();
			GA_RWHandleF density_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DENSITY);
			fpreal rest_density = this->getSolidDensity();
			GA_Offset pt_off;
			GA_FOR_ALL_PTOFF(&gdp, pt_off)
				{
					gdp.setPos3(pt_off, positions[offset_map[pt_off]]);
					density_handle.set(pt_off, rest_density);
				}
		}
	}
}

std::map<UT_String, SIM_Hina_Akinci_Particles *> FetchAllAkinciBoundaries(SIM_Object *fluid_obj)
{
	std::map<UT_String, SIM_Hina_Akinci_Particles *> res;
	SIM_ObjectArray affectors;
	fluid_obj->getAffectors(affectors, "SIM_RelationshipCollide");
	exint num_affectors = affectors.entries();
	for (int i = 0; i < num_affectors; ++i)
	{
		SIM_Object *obj_collider = affectors(i);
		if (obj_collider->getName().equal(fluid_obj->getName()))
			continue;
		UT_String boundary_obj_name = obj_collider->getName();
		SIM_Hina_Akinci_Particles *boundary_akinci = SIM_DATA_GET(*obj_collider, SIM_Hina_Akinci_Particles::DATANAME, SIM_Hina_Akinci_Particles);
		if (boundary_akinci)
			res[boundary_obj_name] = boundary_akinci;
	}
	return res;
}
std::map<UT_String, SIM_Hina_Akinci_Particles *> FetchAllAkinciBoundariesAndApply(SIM_Object *fluid_obj, const std::function<void(SIM_Object *, SIM_Hina_Akinci_Particles *, const UT_String &)> &func)
{
	std::map<UT_String, SIM_Hina_Akinci_Particles *> res;
	SIM_ObjectArray affectors;
	fluid_obj->getAffectors(affectors, "SIM_RelationshipCollide");
	exint num_affectors = affectors.entries();
	for (int i = 0; i < num_affectors; ++i)
	{
		SIM_Object *obj_collider = affectors(i);
		if (obj_collider->getName().equal(fluid_obj->getName()))
			continue;
		UT_String boundary_obj_name = obj_collider->getName();
		SIM_Hina_Akinci_Particles *boundary_akinci = SIM_DATA_GET(*obj_collider, SIM_Hina_Akinci_Particles::DATANAME, SIM_Hina_Akinci_Particles);
		if (boundary_akinci)
		{
			res[boundary_obj_name] = boundary_akinci;
			func(obj_collider, boundary_akinci, boundary_obj_name);
		}
	}
	return res;
}
