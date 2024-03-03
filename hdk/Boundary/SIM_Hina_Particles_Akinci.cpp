#include "SIM_Hina_Particles_Akinci.h"
#include <SIM/SIM_Position.h>

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
	this->x_init = nullptr;
	this->xform = nullptr;
	this->rest_center_of_mass = nullptr;
}
void SIM_Hina_Particles_Akinci::_makeEqual_Particles_Akinci(const SIM_Hina_Particles_Akinci *src)
{
	this->x_init = src->x_init;
	this->xform = src->xform;
	this->rest_center_of_mass = src->rest_center_of_mass;
}
void SIM_Hina_Particles_Akinci::_setup_gdp(GU_Detail *gdp) const
{
	SIM_Hina_Particles::_setup_gdp(gdp);
}


/// For akinci boundaries, we should keep particles init positions(x_init), and moving it ONLY by its transform(xform).
void SIM_Hina_Particles_Akinci::commit()
{
	SIM_Hina_Particles::commit();

	if (x_init == nullptr)
	{
		std::cout << "SIM_Hina_Particles_Akinci::load() called with nullptr" << std::endl;
		return;
	}
	SIM_GeometryAutoWriteLock lock(this);
	GU_Detail &gdp = lock.getGdp();
	GA_RWHandleV3 pos_handle = gdp.getP();

	size_t size = x_init->size();
	if (size == 0)
		return;

	if (gdp.getNumPoints() == x_init->size()) // Update
	{
		GA_Offset pt_off;
		GA_FOR_ALL_PTOFF(&gdp, pt_off)
			{
				size_t pt_idx = offset2index[pt_off];
				UT_Vector3 pos = (*x_init)[pt_idx];
				pos_handle.set(pt_off, pos);
			}
	}
	if (gdp.getNumPoints() == 0) // Add
	{
		for (int pt_idx = 0; pt_idx < size; ++pt_idx)
		{
			GA_Offset pt_off = gdp.appendPoint();
			offset2index[pt_off] = pt_idx;
			index2offset[pt_idx] = pt_off;
			UT_Vector3 pos = (*x_init)[pt_idx];
			pos_handle.set(pt_off, pos);
		}
	}
}

/// Fetch all akinci boundaries from [fluid_obj]
auto FetchAllAkinciBoundaries(SIM_Object *fluid_obj) -> std::vector<SIM_Hina_Particles_Akinci *>
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
			res.emplace_back(boundary_akinci);
	}
	return res;
}

/// Load sop geometry into [x_init] ONLY (ensure [x_init] is already mapped to the fluid solver, aka, [x_init] is not nullptr) (Fluid solver would deal with the `size` problems)
void InitAllAkinciBoundaries(SIM_Object *fluid_obj)
{
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
			(*boundary_akinci->x_init).clear();
			{
				SIM_Geometry *boundary_sop = SIM_DATA_GET(*obj_collider, SIM_GEOMETRY_DATANAME, SIM_Geometry);
				if (!boundary_sop)
					return;
				SIM_GeometryAutoReadLock lock(boundary_sop);
				const GU_Detail *gdp = lock.getGdp();
				(*boundary_akinci->x_init).reserve(gdp->getNumPoints());
				GA_Offset pt_off;
				GA_FOR_ALL_PTOFF(gdp, pt_off)
					{
						UT_Vector3 pos = gdp->getPos3(pt_off);
						(*boundary_akinci->x_init).emplace_back(pos);
					}
			}
		}
	}
}

/// Update Transforms of all Akinci boundaries, and update to [xform] (ensure [xform] is already mapped to the fluid solver, aka, [xform] is not nullptr)
void UpdateAllAkinciBoundaries(SIM_Object *fluid_obj)
{
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
			SIM_Position *position = SIM_DATA_GET(*obj_collider, SIM_POSITION_DATANAME, SIM_Position);
			UT_DMatrix4 xform;
			if (position)
				position->getTransform(xform);
			else
				xform.identity();
			(*boundary_akinci->xform) = xform;
		}
	}
}
