#include "DFSPH2.cuh"

HinaPE::DFSPH2Solver::DFSPH2Solver(HinaPE::real _r, HinaPE::Vector _b) : NeighborBuilder(_r), MaxBound(_b)
{
	Kernel::set_radius(_r);
	constexpr size_t n = 50000;
	Fluid = std::make_shared<DFSPH2FluidGPU>();

	Fluid->x.reserve(n);
	Fluid->v.reserve(n);
	Fluid->a.reserve(n);
	Fluid->m.reserve(n);
	Fluid->V.reserve(n);
	Fluid->rho.reserve(n);
	Fluid->neighbor_this.reserve(n);
	Fluid->neighbor_others.reserve(n);

	Fluid->a.reserve(n);
	Fluid->factor.reserve(n);
	Fluid->k.reserve(n);
	Fluid->density_adv.reserve(n);
}
void HinaPE::DFSPH2Solver::Solve(HinaPE::real dt)
{

}
