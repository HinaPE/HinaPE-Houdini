#ifndef HINAPE_SIM_HINA_DFSPHPARTICLES_H
#define HINAPE_SIM_HINA_DFSPHPARTICLES_H

#include <Particles/SIM_Hina_Particles.h>

SIM_HINA_DERIVED_GEOMETRY_CLASS(
		DFSPHParticles,
		Particles,
		void Commit() override;
		std::map<GA_Offset, fpreal> alpha_cache;
		std::map<GA_Offset, fpreal> kappa_cache;

		void for_all_neighbors(const GA_Offset &pt_off, std::function<void(const GA_Offset &)> func);
)

#endif //HINAPE_SIM_HINA_DFSPHPARTICLES_H
