#include "SIM_Hina_Akinci_Particles.h"

SIM_HINA_DERIVED_GEOMETRY_CLASS_IMPLEMENT(
		Akinci_Particles,
		Particles,
		true,
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
			GA_Offset pt_off;
			for (auto &pair: positions)
			{
				pt_off = gdp.appendPoint();
				this->offset_map[pt_off] = pair.first;
				gdp.setPos3(pt_off, pair.second);
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
			GA_Offset pt_off;
			GA_FOR_ALL_PTOFF(&gdp, pt_off)
				{
					gdp.setPos3(pt_off, positions[offset_map[pt_off]]);
				}
		}
	}
}
