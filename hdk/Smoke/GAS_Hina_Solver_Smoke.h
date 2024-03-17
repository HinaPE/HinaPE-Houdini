#ifndef HINAPE_GAS_HINA_SOLVER_SMOKE_H
#define HINAPE_GAS_HINA_SOLVER_SMOKE_H

#include <SIM_Hina_Generator.h>

#include <Smoke/Smoke.h>

GAS_HINA_SUBSOLVER_CLASS(
		Solver_Smoke,

		std::shared_ptr<HinaPE::SmokeNativeSolver> SmokeNativeSolverPtr;
)

#endif //HINAPE_GAS_HINA_SOLVER_SMOKE_H
