#include "SIM_Hina_SDF_Boundary.h"

#include <Base/utils.h>

SIM_HINA_GEOMETRY_IMPLEMENT(
		SDF_Boundary,
		true,
		HINA_STRING_PARAMETER(TargetGeometryDATANAME, SIM_GEOMETRY_DATANAME) \
        HINA_BOOL_PARAMETER(IsDynamic, true) \
        HINA_FLOAT_PARAMETER(Bounciness, .4) \
        HINA_FLOAT_PARAMETER(Friction, .02) \
)

void SIM_Hina_SDF_Boundary::_init_SDF_Boundary()
{
	this->S = nullptr;
}
void SIM_Hina_SDF_Boundary::_makeEqual_SDF_Boundary(const SIM_Hina_SDF_Boundary *src)
{
	this->S = src->S;
}
void SIM_Hina_SDF_Boundary::_setup_gdp(GU_Detail *gdp) const {}

/// Fetch all akinci boundaries from [fluid_obj]
auto FetchAllSDFBoundaries(SIM_Object *fluid_obj) -> std::vector<SIM_Hina_SDF_Boundary *>
{
	std::vector<SIM_Hina_SDF_Boundary *> res;
	SIM_ObjectArray affectors;
	fluid_obj->getAffectors(affectors, "SIM_RelationshipCollide");
	exint num_affectors = affectors.entries();
	for (int i = 0; i < num_affectors; ++i)
	{
		SIM_Object *obj_collider = affectors(i);
		if (obj_collider->getName().equal(fluid_obj->getName()))
			continue;
		SIM_Hina_SDF_Boundary *SDF_boundary = SIM_DATA_GET(*obj_collider, SIM_Hina_SDF_Boundary::DATANAME, SIM_Hina_SDF_Boundary);
		if (SDF_boundary)
			res.emplace_back(SDF_boundary);
	}
	return res;
}

void InitAllSDFBoundaries(SIM_Object *fluid_obj)
{
	SIM_ObjectArray affectors;
	fluid_obj->getAffectors(affectors, "SIM_RelationshipCollide");
	exint num_affectors = affectors.entries();
	for (int i = 0; i < num_affectors; ++i)
	{
		SIM_Object *obj_collider = affectors(i);
		if (obj_collider->getName().equal(fluid_obj->getName()))
			continue;
		SIM_Hina_SDF_Boundary *SDF_boundary = SIM_DATA_GET(*obj_collider, SIM_Hina_SDF_Boundary::DATANAME, SIM_Hina_SDF_Boundary);
		if (SDF_boundary)
		{
			SDF_boundary->S = nullptr;
			Vector pos;
			std::pair<std::vector<Vector>, std::vector<size_t>> triangle_mesh_info = ReadTriangleMeshFromGeometry<real, Vector>(obj_collider, SDF_boundary->getTargetGeometryDATANAME().c_str(), pos);
			SDF_boundary->S = std::make_shared<Surface>(triangle_mesh_info.first, triangle_mesh_info.second);
			SDF_boundary->S->update_transform(pos, Quaternion(0, 0, 0, 1));
		}
	}
}
