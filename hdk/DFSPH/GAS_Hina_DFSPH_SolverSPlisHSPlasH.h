#ifndef HINAPE_GAS_HINA_DFSPH_SOLVERSPLISHSPLASH_H
#define HINAPE_GAS_HINA_DFSPH_SOLVERSPLISHSPLASH_H

#include <SIM_Hina_Generator.h>
#include <HinaPE/DFSPH/DFSPH.h>

class SIM_Hina_DFSPH_Particles;
class SIM_Hina_Akinci_Particles;

GAS_HINA_SUBSOLVER_CLASS(
		DFSPH_SolverSPlisHSPlasH,

		std::shared_ptr<HinaPE::DFSPHFluidSPlisHSPlasHCPU> SolverPtr;
		bool inited;
)

#endif //HINAPE_GAS_HINA_DFSPH_SOLVERSPLISHSPLASH_H
