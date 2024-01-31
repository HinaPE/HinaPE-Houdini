#ifndef HINAPE_SIM_HINA_DFSPHPARTICLES_H
#define HINAPE_SIM_HINA_DFSPHPARTICLES_H

#include <Common/SIM_Hina_Particles.h>

SIM_HINA_DERIVED_GEOMETRY_CLASS(
		DFSPHParticles,
		Particles,
		void Commit() override;
)

#endif //HINAPE_SIM_HINA_DFSPHPARTICLES_H
