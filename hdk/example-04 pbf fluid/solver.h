#ifndef EXAMPLE_04_SOLVER_H
#define EXAMPLE_04_SOLVER_H

#include "SIM_Interface.h"

namespace HinaPE
{
struct PBFSolver : ISolver
{
	void InitGeometry(SIM_Geometry *geometry) override;
	void SolveGeometry(SIM_Geometry *geometry, const SIM_Time &time) override;

	void InitPosition(SIM_Position *position) override;
	void SolvePosition(SIM_Position *position, const SIM_Time &time) override;
};
}

#endif //EXAMPLE_04_SOLVER_H
