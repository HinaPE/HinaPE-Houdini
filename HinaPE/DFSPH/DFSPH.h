#ifndef HINAPE_DFSPH_H
#define HINAPE_DFSPH_H

#include "common.h"
#include "cuNSearch.h"

namespace HinaPE
{
template<typename Vector3Array, typename ScalarArray, typename Vector3, typename real>
struct DFSPHFluid : public IFluid<Vector3Array, ScalarArray, Vector3, real>
{
	ScalarArray alpha;
	ScalarArray kappa_density;
	ScalarArray kappa_divergence;
	ScalarArray rho_adv;
	ScalarArray d_rho;

	ScalarArray neighbor_this;
	ScalarArray neighbor_others;

	real avg_density_adv;
	real avg_d_density;

	real avg_density_error;
};

template<typename Vector3Array, typename ScalarArray, typename Vector3, typename real>
struct NeighborBuilder
{
	NeighborBuilder(real radius) { searcher = std::make_unique<cuNSearch::NeighborhoodSearch>(radius); }
	void init(const std::vector<Vector3Array *> &x_sets)
	{
		for (const auto &x: x_sets)
			searcher->add_point_set(x->front().data(), x->size(), true, true, true);
	}
	void update_all() { searcher->update_point_sets(); }
	void update_set(int i) { searcher->update_point_set(i); }
	void resize_set(int i, Vector3Array *x) { searcher->resize_point_set(i, x->front().data(), x->size()); }
	void build() { searcher->find_neighbors(); }
	void for_each_neighbor(size_t this_set_idx, size_t target_set_idx, size_t this_pt_idx, const std::function<void(size_t, Vector3)> &f) const
	{
		auto &this_set = searcher->point_set(this_set_idx);
		auto &target_set = searcher->point_set(target_set_idx);
		size_t n = this_set.n_neighbors(target_set_idx, this_pt_idx);
		parallel_for(n, [&](size_t n_idx)
		{
			size_t j = this_set.neighbor(target_set_idx, this_pt_idx, n_idx);
			Vector3 x = {target_set.GetPoints()[3 * j + 0],
						 target_set.GetPoints()[3 * j + 1],
						 target_set.GetPoints()[3 * j + 2]};
			f(j, x);
		});
	}
	size_t n_neighbors(size_t this_set_idx, size_t target_set_idx, int pt_idx)
	{
		const auto &this_set = searcher->point_set(this_set_idx);
		return this_set.n_neighbors(target_set_idx, pt_idx);
	}
private:
	std::unique_ptr<cuNSearch::NeighborhoodSearch> searcher;
};

using DFSPHFluidCPU = DFSPHFluid<CPUVectorArray, CPUScalarArray, Vector, real>;
using DFSPHFluidGPU = DFSPHFluid<GPUVectorArray, GPUScalarArray, Vector, real>;
using NeighborBuilderCPU = NeighborBuilder<CPUVectorArray, CPUScalarArray, Vector, real>;
using NeighborBuilderGPU = NeighborBuilder<GPUVectorArray, GPUScalarArray, Vector, real>;

struct DFSPHSolverCPU
{
	DFSPHSolverCPU(real radius);
	void Init();
	void Solve(real dt);

	void build_neighbors(bool new_fluid_particles = false);
	void for_each_neighbor_fluid(size_t, const std::function<void(size_t, Vector)> &f) const;
	void for_each_neighbor_static_boundary(size_t, const std::function<void(size_t, Vector, size_t)> &f) const;
	void advect_pos(real dt);
	void advect_vel(real dt);
	void compute_non_pressure_force();
	void compute_akinci_volume_mass();
	void compute_density();
	void compute_alpha();
	void correct_density_error(real dt);
	void correct_divergence_error(real dt);

	void compute_density_adv(real dt);
	void compute_density_change(real dt);
	void compute_avg_density_error_den();
	void compute_avg_density_error_div();

	void enforce_boundary();

	DFSPHFluidCPU Fluid;
	std::vector<AkinciBoundaryCPU> StaticBoundaries;
	NeighborBuilderCPU NeighborBuilder;
	real bound;
};

template<typename Vector3Array, typename ScalarArray, typename Vector3, typename real>
struct DFSPHFluidSPlisHSPlasH: public IFluid<Vector3Array, ScalarArray, Vector3, real>
{
	ScalarArray factor;
	ScalarArray density_adv;
	ScalarArray pressure_rho2;
	ScalarArray pressure_rho2_V;
	Vector3Array pressure_acc;

	ScalarArray neighbor_this;
	ScalarArray neighbor_others;
};
using DFSPHFluidSPlisHSPlasHCPU = DFSPHFluidSPlisHSPlasH<CPUVectorArray, CPUScalarArray, Vector, real>;
using DFSPHFluidSPlisHSPlasHGPU = DFSPHFluidSPlisHSPlasH<GPUVectorArray, GPUScalarArray, Vector, real>;

struct DFSPHSolverSPlisHSPlasH
{
	DFSPHSolverSPlisHSPlasH(real radius);

	void build_neighbors();
	void for_each_neighbor_fluid(size_t, const std::function<void(size_t, Vector)> &f) const;
	void for_each_neighbor_static_boundary(size_t, const std::function<void(size_t, Vector, size_t)> &f) const;

	void precompute_values();
	void compute_densities();
	void compute_DFSPH_factors();
	void divergence_solve(real dt);
	void compute_non_pressure_force();
	void update_CFL_timestep();
	void advect_vel();
	void pressure_solve();
	void advect_pos();

	DFSPHFluidSPlisHSPlasHCPU Fluid;
	std::vector<AkinciBoundaryCPU> StaticBoundaries;
	NeighborBuilderCPU NeighborBuilder;
};
}

#endif //HINAPE_DFSPH_H
