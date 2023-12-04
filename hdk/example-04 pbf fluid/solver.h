#ifndef EXAMPLE_04_SOLVER_H
#define EXAMPLE_04_SOLVER_H

#include "SIM_Interface.h"

#include "GAS/GAS_PBDSolve.h"

namespace HinaPE
{
struct PBFSolver final: ISolver
{
	void InitGeometry(SIM_Geometry *geometry) final;
	void SolveGeometry(SIM_Geometry *geometry, const SIM_Time &time) final;
};
}

#endif //EXAMPLE_04_SOLVER_H
