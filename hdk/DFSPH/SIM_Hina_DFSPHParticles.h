#ifndef HINAPE_SIM_HINA_DFSPHPARTICLES_H
#define HINAPE_SIM_HINA_DFSPHPARTICLES_H

#include <Common/SIM_Hina_Particles.h>

SIM_HINA_DERIVED_GEOMETRY_CLASS(
		DFSPHParticles,
		Particles,
		void Commit() override;
		std::map<GA_Offset, fpreal> alpha_cache;
		std::map<GA_Offset, fpreal> kappa_cache;
)

#endif //HINAPE_SIM_HINA_DFSPHPARTICLES_H
