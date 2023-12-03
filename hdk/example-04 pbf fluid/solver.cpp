#include "solver.h"

ImplementInnerSolverRequired(HinaPE::PBFSolver)

void HinaPE::PBFSolver::InitGeometry(SIM_Geometry *geometry)
{
	if (!geometry)
		return;

	Param.Vector3Params["gravity"];
}
void HinaPE::PBFSolver::SolveGeometry(SIM_Geometry *geometry, const SIM_Time &time)
{
	if (!geometry)
		return;
}

void HinaPE::PBFSolver::InitPosition(SIM_Position *position)
{
	if (!position)
		return;
}
void HinaPE::PBFSolver::SolvePosition(SIM_Position *position, const SIM_Time &time)
{
	if (!position)
		return;
}
