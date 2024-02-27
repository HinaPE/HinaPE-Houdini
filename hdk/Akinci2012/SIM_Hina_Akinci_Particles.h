#ifndef HINAPE_SIM_HINA_AKINCI_PARTICLES_H
#define HINAPE_SIM_HINA_AKINCI_PARTICLES_H

#include <Base/SIM_Hina_Particles.h>

SIM_HINA_DERIVED_GEOMETRY_CLASS(
		Akinci_Particles,
		Particles,
		void load_sop(SIM_Object *boundary_obj);

		std::map<GA_Offset, GA_Offset> offset_map; // key: this particles set (Boundary Hina Particles), value: boundary particles set (From SOP)
		bool _dynamic;
		bool _inited;
)

#endif //HINAPE_SIM_HINA_AKINCI_PARTICLES_H
