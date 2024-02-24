#include "SIM_Hina_Akinci2012BoundaryParticles.h"
#include <CUDA_HinaPE/kernels.h>

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
void SIM_Hina_Akinci2012BoundaryParticles::load_sop(SIM_Object *boundary_obj)
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
void SIM_Hina_Akinci2012BoundaryParticles::calculate_mass()
{
	SIM_GeometryAutoWriteLock lock(this);
	GU_Detail &gdp = lock.getGdp();
	GA_RWHandleF mass_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_MASS);
	GA_ROHandleF volume_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VOLUME);
	GA_Offset pt_off;
	GA_FOR_ALL_PTOFF(&gdp, pt_off)
		{
			fpreal volume = volume_handle.get(pt_off);
			mass_handle.set(pt_off, volume * 1000.);
		}
}
void SIM_Hina_Akinci2012BoundaryParticles::calculate_volume()
{
	HinaPE::CubicSplineKernel<false> kernel(getTargetSpacing() * getKernelRadiusOverTargetSpacing());
	SIM_GeometryAutoWriteLock lock(this);
	GU_Detail &gdp = lock.getGdp();
	GA_RWHandleF volume_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VOLUME);

	GA_Offset pt_off;
	GA_FOR_ALL_PTOFF(&gdp, pt_off)
		{
			fpreal volume = 0.0;
			volume += kernel.kernel(0); // remember to include self (self is also a neighbor of itself)
			UT_Vector3 p_i = gdp.getPos3(pt_off);
			for_each_neighbor_self(pt_off, [&](const GA_Offset &n_off, const UT_Vector3 &n_pos)
			{
				UT_Vector3 p_j = n_pos;
				const UT_Vector3 r = p_i - p_j;
				volume += kernel.kernel(r.length());
			});
			for_each_neighbor_others(pt_off, [&](const GA_Offset &n_off, const UT_Vector3 &n_pos)
			{
				UT_Vector3 p_j = n_pos;
				const UT_Vector3 r = p_i - p_j;
				volume += kernel.kernel(r.length());
			});
			volume = 1.0 / volume;
			volume_handle.set(pt_off, volume);
		}
}
std::map<UT_String, SIM_Hina_Akinci2012BoundaryParticles *> FetchAllAkinciBoundaries(SIM_Object *fluid_obj)
{
	std::map<UT_String, SIM_Hina_Akinci2012BoundaryParticles *> res;
	SIM_ObjectArray affectors;
	fluid_obj->getAffectors(affectors, "SIM_RelationshipCollide");
	exint num_affectors = affectors.entries();
	for (int i = 0; i < num_affectors; ++i)
	{
		SIM_Object *obj_collider = affectors(i);
		if (obj_collider->getName().equal(fluid_obj->getName()))
			continue;
		UT_String boundary_obj_name = obj_collider->getName();
		SIM_Hina_Akinci2012BoundaryParticles *boundary_akinci = SIM_DATA_GET(*obj_collider, SIM_Hina_Akinci2012BoundaryParticles::DATANAME, SIM_Hina_Akinci2012BoundaryParticles);
		if (boundary_akinci)
			res[boundary_obj_name] = boundary_akinci;
	}
	return res;
}
std::map<UT_String, SIM_Hina_Akinci2012BoundaryParticles *> FetchAllAkinciBoundariesAndApply(SIM_Object *fluid_obj, const std::function<void(SIM_Object *, SIM_Hina_Akinci2012BoundaryParticles *, const UT_String &)> &func)
{
	std::map<UT_String, SIM_Hina_Akinci2012BoundaryParticles *> res;
	SIM_ObjectArray affectors;
	fluid_obj->getAffectors(affectors, "SIM_RelationshipCollide");
	exint num_affectors = affectors.entries();
	for (int i = 0; i < num_affectors; ++i)
	{
		SIM_Object *obj_collider = affectors(i);
		if (obj_collider->getName().equal(fluid_obj->getName()))
			continue;
		UT_String boundary_obj_name = obj_collider->getName();
		SIM_Hina_Akinci2012BoundaryParticles *boundary_akinci = SIM_DATA_GET(*obj_collider, SIM_Hina_Akinci2012BoundaryParticles::DATANAME, SIM_Hina_Akinci2012BoundaryParticles);
		if (boundary_akinci)
		{
			res[boundary_obj_name] = boundary_akinci;
			func(obj_collider, boundary_akinci, boundary_obj_name);
		}
	}
	return res;
}
