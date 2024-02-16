#ifndef HINAPE_SIM_HINA_AKINCI2012BOUNDARYPARTICLES_H
#define HINAPE_SIM_HINA_AKINCI2012BOUNDARYPARTICLES_H

#include <Particles/SIM_Hina_Particles.h>

#include "SIM/SIM_Geometry.h"

SIM_HINA_DERIVED_GEOMETRY_CLASS(
		Akinci2012BoundaryParticles,
		Particles,
		void UpdateBoundaryParticles(SIM_Object *boundary_obj);

		std::map<GA_Offset, GA_Offset> offset_map;
		bool _dynamic;
		bool _init;
)

#endif //HINAPE_SIM_HINA_AKINCI2012BOUNDARYPARTICLES_H