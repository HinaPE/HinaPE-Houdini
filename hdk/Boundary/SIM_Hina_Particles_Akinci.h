#ifndef HINAPE_SIM_HINA_PARTICLES_AKINCI_H
#define HINAPE_SIM_HINA_PARTICLES_AKINCI_H

#include <Base/SIM_Hina_Particles.h>

SIM_HINA_DERIVED_GEOMETRY_CLASS(
		Particles_Akinci,
		Particles,
		HINA_GETSET_PARAMETER(SolidDensity, GETSET_DATA_FUNCS_F)
		HINA_GETSET_PARAMETER(IsDynamic, GETSET_DATA_FUNCS_B)
		void load_sop(SIM_Object *boundary_obj);

		bool _inited;
)

std::vector<SIM_Hina_Particles_Akinci *> FetchAllAkinciBoundaries(SIM_Object *fluid_obj);

#endif //HINAPE_SIM_HINA_PARTICLES_AKINCI_H
