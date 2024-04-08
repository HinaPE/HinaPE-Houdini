#ifndef HINAPE_SIM_HINA_PARTICLES_AKINCI_H
#define HINAPE_SIM_HINA_PARTICLES_AKINCI_H

#include <Base/SIM_Hina_Particles.h>

SIM_HINA_DERIVED_GEOMETRY_CLASS(
		Particles_Akinci,
		Particles,
		HINA_GETSET_PARAMETER(TargetGeometryDATANAME, GETSET_DATA_FUNCS_S) \
		HINA_GETSET_PARAMETER(SolidDensity, GETSET_DATA_FUNCS_F)
		HINA_GETSET_PARAMETER(Buoyancy, GETSET_DATA_FUNCS_F)
        HINA_GETSET_PARAMETER(RotationStiffness,GETSET_DATA_FUNCS_F)
		HINA_GETSET_PARAMETER(IsDynamic, GETSET_DATA_FUNCS_B)

        void commit() override;

		VectorArrayCPU *x_init, *normals, *u_diff;
		UT_DMatrix4 *xform;
		Vector center_of_mass;

        std::vector<int> *SP;

		int b_set_index; // Cache, FOR FLUID SOLVER
)

auto FetchAllAkinciBoundaries(SIM_Object *fluid_obj) -> std::vector<SIM_Hina_Particles_Akinci *>;
void InitAllAkinciBoundaries(SIM_Object *fluid_obj);
void UpdateAllAkinciBoundaries(SIM_Object *fluid_obj);

#endif //HINAPE_SIM_HINA_PARTICLES_AKINCI_H
