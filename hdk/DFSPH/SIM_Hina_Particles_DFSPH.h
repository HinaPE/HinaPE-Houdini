#ifndef HINAPE_SIM_HINA_PARTICLES_DFSPH_H
#define HINAPE_SIM_HINA_PARTICLES_DFSPH_H

#include <Base/SIM_Hina_Particles.h>

SIM_HINA_DERIVED_GEOMETRY_CLASS(
		Particles_DFSPH,
		Particles,
		void commit() override;

		ScalarArrayCPU *factor, *density_adv;
)

#endif //HINAPE_SIM_HINA_PARTICLES_DFSPH_H
