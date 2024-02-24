#ifndef HINAPE_SIM_HINA_DFSPHPARTICLES_H
#define HINAPE_SIM_HINA_DFSPHPARTICLES_H

#include <Particles/SIM_Hina_Particles.h>

SIM_HINA_DERIVED_GEOMETRY_CLASS(
		DFSPHParticles,
		Particles,
		std::map<GA_Offset, fpreal> alpha_cache;
		std::map<GA_Offset, fpreal> kappa_cache;
		std::map<GA_Offset, fpreal> Drho_cache;
		void commit() override;
)

#endif //HINAPE_SIM_HINA_DFSPHPARTICLES_H
