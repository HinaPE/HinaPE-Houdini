#include "SIM_Hina_Akinci2012BoundaryParticles.h"

SIM_HINA_DERIVED_GEOMETRY_CLASS_IMPLEMENT(
		Akinci2012BoundaryParticles,
		Particles,
		true,
		TARGET_PARTICLE_GEOMETRY(SIM_Hina_Akinci2012BoundaryParticles)
)

void SIM_Hina_Akinci2012BoundaryParticles::_init_Akinci2012BoundaryParticles()
{
	this->_init = true;
	this->_dynamic = true;
	this->offset_map.clear();
}
void SIM_Hina_Akinci2012BoundaryParticles::_makeEqual_Akinci2012BoundaryParticles(const SIM_Hina_Akinci2012BoundaryParticles *src)
{
	this->_init = src->_init;
	this->_dynamic = src->_dynamic;
	this->offset_map = src->offset_map;
}
void SIM_Hina_Akinci2012BoundaryParticles::_setup_gdp(GU_Detail *gdp) const { SIM_Hina_Particles::_setup_gdp(gdp); }
void SIM_Hina_Akinci2012BoundaryParticles::UpdateBoundaryParticles(SIM_Object *boundary_obj)
{
	// Reload the whole boundary particles
	if (_init)
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
				gdp.setPos3(pt_off, pair.second);
				this->offset_map[pt_off] = pair.first;
			}
		}
		_init = false;
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
