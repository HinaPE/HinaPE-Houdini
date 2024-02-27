#ifndef HINAPE_HOUDINI_GAS_HINA_DFSPH_SOLVER_H
#define HINAPE_HOUDINI_GAS_HINA_DFSPH_SOLVER_H

#include <SIM_Hina_Generator.h>
#include <HinaPE/DFSPH/DFSPH.h>

class SIM_Hina_DFSPH_Particles;
class SIM_Hina_Akinci_Particles;

GAS_HINA_SUBSOLVER_CLASS(
		DFSPH_Solver,

		std::shared_ptr<HinaPE::DFSPHSolverCPU> SolverPtr;
		bool inited;
)

#endif //HINAPE_HOUDINI_GAS_HINA_DFSPH_SOLVER_H
