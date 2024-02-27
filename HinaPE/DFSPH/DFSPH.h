#ifndef HINAPE_DFSPH_H
#define HINAPE_DFSPH_H

#include "common.h"
#include "kernels.h"
#include "cuNSearch.h"

namespace HinaPE
{

void parallel_for(size_t n, const std::function<void(size_t)> &f);
void serial_for(size_t n, const std::function<void(size_t)> &f);

template<typename Vector3Array, typename ScalarArray, typename Vector3, typename real>
struct DFSPHFluid
{
	size_t size;
	Vector3Array x;
	Vector3Array v;
	Vector3Array f;
	ScalarArray m;
	ScalarArray inv_m;
	ScalarArray V;
	ScalarArray rho;
	ScalarArray alpha;
	ScalarArray kappa_density;
	ScalarArray kappa_divergence;
	ScalarArray rho_adv;
	ScalarArray d_rho;

	real avg_density_adv;
	real avg_d_density;
};

template<typename Vector3Array, typename ScalarArray, typename Vector3, typename real>
struct AkinciBoundary
{
	size_t size;
	Vector3Array x;
	Vector3Array v;
	ScalarArray m;
	ScalarArray V;
	ScalarArray rho;
};

template<typename Vector3Array, typename ScalarArray, typename Vector3, typename real>
struct NeighborBuilder
{
	NeighborBuilder(real radius) { searcher = std::make_unique<cuNSearch::NeighborhoodSearch>(radius); }
	void init(const std::vector<Vector3Array *> &x_sets, real radius)
	{
		for (const auto &x: x_sets)
			searcher->add_point_set(x->front().data(), x->size(), true, true, true);
	}
	void update_all() { searcher->update_point_sets(); }
	void update_set(int i) { searcher->update_point_set(i); }
	void resize_set(int i, Vector3Array *x) { searcher->resize_point_set(i, x->front().data(), x->size()); }
	void build() { searcher->find_neighbors(); }
	void for_each_neighbor(size_t this_set_idx, size_t target_set_idx, const std::function<void(size_t, Vector3)> &f) const
	{
		auto &this_set = searcher->point_set(this_set_idx);
		auto &target_set = searcher->point_set(target_set_idx);
		parallel_for(this_set.n_points(), [&](size_t i)
		{
			size_t n = this_set.n_neighbors(target_set_idx, i);
			serial_for(n, [&](size_t _)
			{
				size_t j = this_set.neighbor(target_set_idx, i, _);
				Vector3 x = {target_set.GetPoints()[3 * j + 0],
							 target_set.GetPoints()[3 * j + 1],
							 target_set.GetPoints()[3 * j + 2]};
				f(j, x);
			});
		});
	}

//	size_t neighbor_sum(int i)
//	{
//		auto &set = searcher->point_set(i);
//		size_t sum = 0;
//		for (size_t j = 0; j < set.n_points(); ++j)
//			sum += set.n_neighbors(0, j);
//		return sum;
//	}
private:
	std::unique_ptr<cuNSearch::NeighborhoodSearch> searcher;
};

using DFSPHFluidCPU = DFSPHFluid<CPUVectorArray, CPUScalarArray, Vector, real>;
using DFSPHFluidGPU = DFSPHFluid<GPUVectorArray, GPUScalarArray, Vector, real>;
using AkinciBoundaryCPU = AkinciBoundary<CPUVectorArray, CPUScalarArray, Vector, real>;
using AkinciBoundaryGPU = AkinciBoundary<GPUVectorArray, GPUScalarArray, Vector, real>;
using NeighborBuilderCPU = NeighborBuilder<CPUVectorArray, CPUScalarArray, Vector, real>;
using NeighborBuilderGPU = NeighborBuilder<GPUVectorArray, GPUScalarArray, Vector, real>;

struct DFSPHSolverCPU
{
	DFSPHSolverCPU(real radius);
	void Init(const CPUVectorArray &fluid, const CPUVectorArray &static_boundary);
	void Solve(real dt);

	void build_neighbors(bool new_fluid_particles = false);
	void for_each_neighbor_fluid(const std::function<void(size_t, Vector)> &f) const;
	void for_each_neighbor_static_boundary(const std::function<void(size_t, Vector)> &f) const;
	void advect_pos(real dt);
	void advect_vel(real dt);
	void compute_non_pressure_force();
	void compute_density();
	void compute_alpha();
	void compute_kappa_density(real dt);
	void compute_kappa_divergence(real dt);
	void compute_density_error(real dt);
	void correct_density_error(real dt);
	void correct_divergence_error(real dt);

	DFSPHFluidCPU Fluid;
	AkinciBoundaryCPU StaticBoundary;
	NeighborBuilderCPU NeighborBuilder;
	CubicSplineKernel<false> Kernel;
};
}

#endif //HINAPE_DFSPH_H
