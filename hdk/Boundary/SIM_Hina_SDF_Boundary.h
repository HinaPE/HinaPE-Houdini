#ifndef HINAPE_SIM_HINA_SDF_BOUNDARY_H
#define HINAPE_SIM_HINA_SDF_BOUNDARY_H

#include <SIM_Hina_Generator.h>
#include <HinaPE/common/geometry.h>
#include <memory>

using real = float;
using Vector = UT_Vector3;
using Quaternion = UT_Quaternion;
using Surface = HinaPE::ISurface<real, Vector, Quaternion>;

SIM_HINA_GEOMETRY_CLASS(
		SDF_Boundary,
		HINA_GETSET_PARAMETER(TargetGeometryDATANAME, GETSET_DATA_FUNCS_S) \
		HINA_GETSET_PARAMETER(IsDynamic, GETSET_DATA_FUNCS_B) \
		HINA_GETSET_PARAMETER(Bounciness, GETSET_DATA_FUNCS_F) \
		HINA_GETSET_PARAMETER(Friction, GETSET_DATA_FUNCS_F)

		std::shared_ptr<Surface> S;
)

auto FetchAllSDFBoundaries(SIM_Object *fluid_obj) -> std::vector<SIM_Hina_SDF_Boundary *>;
void InitAllSDFBoundaries(SIM_Object *fluid_obj);

// if SDF boundary is linked as rigidbody, then the transform is updated by rigidbody
//void UpdateAllSDFBoundaries(SIM_Object *fluid_obj);

#endif //HINAPE_SIM_HINA_SDF_BOUNDARY_H
