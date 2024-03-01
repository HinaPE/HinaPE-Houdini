#include "DFSPH_CUDA.h"

HinaPE_CUDA::DFSPH_AkinciSolverGPU::DFSPH_AkinciSolverGPU(HinaPE_CUDA::real _r, HinaPE_CUDA::Vector _b)
		: NeighborBuilder(_r), MaxBound(_b / 2.), VolumeInited(false)
{

}
void HinaPE_CUDA::DFSPH_AkinciSolverGPU::Solve(HinaPE_CUDA::real dt)
{
	build_neighbors();
	compute_density();
	compute_factor();
	divergence_solve(dt);
	non_pressure_force();
	predict_velocity(dt);
	pressure_solve(dt);
	advect(dt);
	enforce_boundary();
}
void HinaPE_CUDA::DFSPH_AkinciSolverGPU::build_neighbors() {}
void HinaPE_CUDA::DFSPH_AkinciSolverGPU::compute_density() {}
void HinaPE_CUDA::DFSPH_AkinciSolverGPU::compute_factor() {}
void HinaPE_CUDA::DFSPH_AkinciSolverGPU::divergence_solve(HinaPE_CUDA::real dt) {}
void HinaPE_CUDA::DFSPH_AkinciSolverGPU::non_pressure_force() {}
void HinaPE_CUDA::DFSPH_AkinciSolverGPU::predict_velocity(HinaPE_CUDA::real dt) {}
void HinaPE_CUDA::DFSPH_AkinciSolverGPU::pressure_solve(HinaPE_CUDA::real dt) {}
void HinaPE_CUDA::DFSPH_AkinciSolverGPU::advect(HinaPE_CUDA::real dt) {}
void HinaPE_CUDA::DFSPH_AkinciSolverGPU::enforce_boundary() {}
void HinaPE_CUDA::DFSPH_AkinciSolverGPU::_for_each_fluid_particle(const std::function<void(size_t, Vector)> &) {}
void HinaPE_CUDA::DFSPH_AkinciSolverGPU::_for_each_neighbor_fluid(size_t, const std::function<void(size_t, Vector)> &) {}
void HinaPE_CUDA::DFSPH_AkinciSolverGPU::_for_each_neighbor_boundaries(size_t, const std::function<void(size_t, Vector, size_t)> &) {}
void HinaPE_CUDA::DFSPH_AkinciSolverGPU::_resize() {}
void HinaPE_CUDA::DFSPH_AkinciSolverGPU::_compute_akinci_volume() {}
void HinaPE_CUDA::DFSPH_AkinciSolverGPU::_compute_density_change() {}
void HinaPE_CUDA::DFSPH_AkinciSolverGPU::_compute_density_adv(HinaPE_CUDA::real dt) {}
HinaPE_CUDA::real HinaPE_CUDA::DFSPH_AkinciSolverGPU::_compute_density_error(const HinaPE_CUDA::real offset) { return 0; }
HinaPE_CUDA::real HinaPE_CUDA::DFSPH_AkinciSolverGPU::_divergence_solver_iteration(HinaPE_CUDA::real dt) { return 0; }
HinaPE_CUDA::real HinaPE_CUDA::DFSPH_AkinciSolverGPU::_pressure_solve_iteration(HinaPE_CUDA::real dt) { return 0; }
void HinaPE_CUDA::DFSPH_AkinciSolverGPU::_divergence_solver_iteration_kernel(HinaPE_CUDA::real dt) {}
void HinaPE_CUDA::DFSPH_AkinciSolverGPU::_pressure_solve_iteration_kernel(HinaPE_CUDA::real dt) {}

__global__ void foo()
{
	printf("I am from CUDA");
}

void HinaPE_CUDA::test1()
{
	foo<<<1, 10>>>();
	cudaDeviceSynchronize();
}
