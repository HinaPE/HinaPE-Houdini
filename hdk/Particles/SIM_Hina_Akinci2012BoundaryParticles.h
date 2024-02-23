#ifndef HINAPE_SIM_HINA_AKINCI2012BOUNDARYPARTICLES_H
#define HINAPE_SIM_HINA_AKINCI2012BOUNDARYPARTICLES_H

#include <Particles/SIM_Hina_Particles.h>

#include "SIM/SIM_Geometry.h"

SIM_HINA_DERIVED_GEOMETRY_CLASS(
		Akinci2012BoundaryParticles,
		Particles,
		void UpdateBoundaryParticlesFromSOP(SIM_Object *boundary_obj);
		void calculate_volume() override; // call after neighbor list is built

		std::map<GA_Offset, GA_Offset> offset_map; // key: this particles set (Boundary Hina Particles), value: boundary particles set (From SOP)
		bool _dynamic;
		bool _init;
)

#endif //HINAPE_SIM_HINA_AKINCI2012BOUNDARYPARTICLES_H
