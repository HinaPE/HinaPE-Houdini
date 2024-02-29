#include "DFSPH_Bender.h"

HinaPE::DFSPH_BenderSolver::DFSPH_BenderSolver(HinaPE::real _r, HinaPE::Vector _b) : NeighborBuilder(_r), MaxBound(_b), VolumeInited(false)
{
	Fluid = std::make_shared<DFSPH_BenderFluidCPU>();
}
void HinaPE::DFSPH_BenderSolver::Solve(HinaPE::real dt)
{

}
