#ifndef HINAPE_SIM_HINA_PARTICLES_PBF_H
#define HINAPE_SIM_HINA_PARTICLES_PBF_H

#include <Base/SIM_Hina_Particles.h>

SIM_HINA_DERIVED_GEOMETRY_CLASS(
		Particles_PBF,
		Particles,
		void commit() override;

		VectorArrayCPU *p_x;
)

#endif //HINAPE_SIM_HINA_PARTICLES_PBF_H
