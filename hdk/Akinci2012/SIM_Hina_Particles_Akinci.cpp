#include "SIM_Hina_Particles_Akinci.h"

SIM_HINA_DERIVED_GEOMETRY_CLASS_IMPLEMENT(
		Particles_Akinci,
		Particles,
		true,
		HINA_FLOAT_PARAMETER(SolidDensity, 1000.) \
        HINA_BOOL_PARAMETER(IsDynamic, false) \
        TARGET_PARTICLE_GEOMETRY(SIM_Hina_Particles_Akinci)
)
void SIM_Hina_Particles_Akinci::_init_Particles_Akinci()
{
	this->_inited = false;
}
void SIM_Hina_Particles_Akinci::_makeEqual_Particles_Akinci(const SIM_Hina_Particles_Akinci *src)
{
	this->_inited = src->_inited;
}
void SIM_Hina_Particles_Akinci::_setup_gdp(GU_Detail *gdp) const
{
	SIM_Hina_Particles::_setup_gdp(gdp);
}
void SIM_Hina_Particles_Akinci::load_sop(SIM_Object *boundary_obj)
{
	if (!_inited)
	{
		// Reload the whole boundary particles
		std::vector<UT_Vector3> positions;
		positions.clear();
		{
			SIM_Geometry *boundary_sop = SIM_DATA_GET(*boundary_obj, SIM_GEOMETRY_DATANAME, SIM_Geometry);
			if (!boundary_sop)
				return;
			SIM_GeometryAutoReadLock lock(boundary_sop);
			const GU_Detail *gdp = lock.getGdp();
			positions.reserve(gdp->getNumPoints());
			GA_Offset pt_off;
			GA_FOR_ALL_PTOFF(gdp, pt_off)
				{
					UT_Vector3 pos = gdp->getPos3(pt_off);
					positions.emplace_back(pos);
				}
		}

		{
			SIM_GeometryAutoWriteLock lock(this);
			GU_Detail &gdp = lock.getGdp();
			GA_Offset pt_off;
			for (auto &pos: positions)
			{
				pt_off = gdp.appendPoint();
				gdp.setPos3(pt_off, pos);
			}
		}
		_inited = true;
		return;
	}
}

std::vector<SIM_Hina_Particles_Akinci *> FetchAllAkinciBoundaries(SIM_Object *fluid_obj)
{
	std::vector<SIM_Hina_Particles_Akinci *> res;
	SIM_ObjectArray affectors;
	fluid_obj->getAffectors(affectors, "SIM_RelationshipCollide");
	exint num_affectors = affectors.entries();
	for (int i = 0; i < num_affectors; ++i)
	{
		SIM_Object *obj_collider = affectors(i);
		if (obj_collider->getName().equal(fluid_obj->getName()))
			continue;
		UT_String boundary_obj_name = obj_collider->getName();
		SIM_Hina_Particles_Akinci *boundary_akinci = SIM_DATA_GET(*obj_collider, SIM_Hina_Particles_Akinci::DATANAME, SIM_Hina_Particles_Akinci);
		if (boundary_akinci)
		{
			if (!boundary_akinci->_inited)
				boundary_akinci->load_sop(obj_collider);
			res.emplace_back(boundary_akinci);
		}
	}
	return res;
}
