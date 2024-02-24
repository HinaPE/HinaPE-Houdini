#ifndef HINAPE_SIM_HINA_AKINCI2012BOUNDARYPARTICLES_H
#define HINAPE_SIM_HINA_AKINCI2012BOUNDARYPARTICLES_H

#include <Particles/SIM_Hina_Particles.h>

#include "SIM/SIM_Geometry.h"

SIM_HINA_DERIVED_GEOMETRY_CLASS(
		Akinci2012BoundaryParticles,
		Particles,
		void load_sop(SIM_Object *boundary_obj);
		void calculate_mass() override; // call after volume is calculated
		void calculate_volume() override; // call after neighbor list is built

		std::map<GA_Offset, GA_Offset> offset_map; // key: this particles set (Boundary Hina Particles), value: boundary particles set (From SOP)
		bool _dynamic;
		bool _init;
)

// Utility
std::map<UT_String, SIM_Hina_Akinci2012BoundaryParticles *> FetchAllAkinciBoundaries(SIM_Object *fluid_obj);
std::map<UT_String, SIM_Hina_Akinci2012BoundaryParticles *> FetchAllAkinciBoundariesAndApply(SIM_Object *fluid_obj, const std::function<void(SIM_Object *obj_boundary, SIM_Hina_Akinci2012BoundaryParticles *boundary_akinci, const UT_String &boundary_name)> &func);

#endif //HINAPE_SIM_HINA_AKINCI2012BOUNDARYPARTICLES_H
