#ifndef HINAPE_DFSPH_CUDA_H
#define HINAPE_DFSPH_CUDA_H

/**
 * DFSPH implementation, Native CUDA 12.1
 */

#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/universal_vector.h>
#include <thrust/for_each.h>
#include <Eigen/Dense>
#include "cuNSearch.h"
namespace HinaPE_CUDA
{
inline void serial_for(size_t n, const std::function<void(size_t)> &f) { for (size_t i = 0; i < n; ++i) { f(i); }}

template<typename real, typename Vector3, typename ScalarArray, typename Vector3Array>
struct INeighborBuilder
{
	virtual void init(const std::vector<Vector3Array *> &x_sets) = 0;
	virtual void update_set(int i) = 0;
	virtual void resize_set(int i, Vector3Array *x) = 0;
	virtual void disable_set_to_search_from(int i) = 0;
	virtual void build() = 0;
	virtual void for_each_neighbor(size_t this_set_idx, size_t target_set_idx, size_t this_pt_idx, const std::function<void(size_t, Vector3)> &f) const = 0;
	virtual size_t n_neighbors(size_t this_set_idx, size_t target_set_idx, int pt_idx) = 0;
};
template<typename real, typename Vector3, typename ScalarArray, typename Vector3Array>
struct NeighborBuilderGPU : INeighborBuilder<real, Vector3, ScalarArray, Vector3Array>
{
	NeighborBuilderGPU(real radius) { searcher = std::make_shared<cuNSearch::NeighborhoodSearch>(radius); }
	void init(const std::vector<Vector3Array *> &x_sets) override { for (const auto &x: x_sets) searcher->add_point_set(x->front().data(), x->size(), true, true, true); }
	void update_set(int i) override { searcher->update_point_set(i); }
	void disable_set_to_search_from(int i) override { searcher->set_active(i, false /* search neighbors */, true /* BE SEARCHED by other points sets*/); }
	void resize_set(int i, Vector3Array *x) override { searcher->resize_point_set(i, x->front().data(), x->size()); }
	void build() override { searcher->find_neighbors(); }
	void for_each_neighbor(size_t this_set_idx, size_t target_set_idx, size_t this_pt_idx, const std::function<void(size_t, Vector3)> &f) const override
	{
		auto &this_set = searcher->point_set(this_set_idx);
		auto &target_set = searcher->point_set(target_set_idx);
		size_t n = this_set.n_neighbors(target_set_idx, this_pt_idx);
		serial_for(n, [&](size_t n_idx)
		{
			size_t j = this_set.neighbor(target_set_idx, this_pt_idx, n_idx);
			Vector3 x = {target_set.GetPoints()[3 * j + 0],
						 target_set.GetPoints()[3 * j + 1],
						 target_set.GetPoints()[3 * j + 2]};
			f(j, x);
		});
	}
	size_t n_neighbors(size_t this_set_idx, size_t target_set_idx, int pt_idx) override
	{
		const auto &this_set = searcher->point_set(this_set_idx);
		return this_set.n_neighbors(target_set_idx, pt_idx);
	}
private:
	std::shared_ptr<cuNSearch::NeighborhoodSearch> searcher;
};

template<typename real, typename Vector3, typename ScalarArray, typename Vector3Array>
struct IFluid
{
	size_t size;
	Vector3Array x;
	Vector3Array v;
	Vector3Array a;
	ScalarArray m;
	ScalarArray V;
	ScalarArray rho;
	ScalarArray neighbor_this;
	ScalarArray neighbor_others;
};

template<typename real, typename Vector3, typename ScalarArray, typename Vector3Array>
struct IAkinciBoundary
{
	size_t size;
	Vector3Array x;
	Vector3Array v;
	Vector3Array a;
	ScalarArray m;
	ScalarArray V;
	ScalarArray rho;

	ScalarArray neighbor_this;
	ScalarArray neighbor_others;
};

using real = float;
using Vector = Eigen::Vector3<real>;
using ScalarArrayCPU = thrust::host_vector<real>;
using VectorArrayCPU = thrust::host_vector<Vector>;
using ScalarArrayGPU = thrust::universal_vector<real>;
using VectorArrayGPU = thrust::universal_vector<Vector>;
using FluidGPU = IFluid<real, Vector, ScalarArrayGPU, VectorArrayGPU>;
using AkinciBoundaryCPU = IAkinciBoundary<real, Vector, ScalarArrayGPU, VectorArrayGPU>;
using NeighborBuilder = NeighborBuilderGPU<real, Vector, std::vector<real>, std::vector<Vector>>;

struct DFSPH_AkinciFluidGPU : public FluidGPU
{
	ScalarArrayGPU factor;
	ScalarArrayGPU k;
	ScalarArrayGPU density_adv;
};

struct DFSPH_AkinciParamGPU
{
	real FLUID_REST_DENSITY = 1000.0f;
	std::vector<real> BOUNDARY_REST_DENSITY;
	real FLUID_PARTICLE_RADIUS = 0.01;
	real FLUID_SURFACE_TENSION = 0.01;
	real FLUID_VISCOSITY = 0.01;
	real BOUNDARY_VISCOSITY = 0;
	Vector GRAVITY = Vector(0, -9.8, 0);
	bool TOP_OPEN = true;
};

struct DFSPH_AkinciSolverGPU : public DFSPH_AkinciParamGPU
{
	DFSPH_AkinciSolverGPU(real, Vector);
	void Solve(real dt);
	std::shared_ptr<DFSPH_AkinciFluidGPU> Fluid;
	std::vector<std::shared_ptr<AkinciBoundaryCPU>> Boundaries;

protected:
	void build_neighbors();
	void compute_density();
	void compute_factor();
	void divergence_solve(real dt);
	void non_pressure_force();
	void predict_velocity(real dt);
	void pressure_solve(real dt);
	void advect(real dt);
	void enforce_boundary();

private:
	void _for_each_fluid_particle(const std::function<void(size_t, Vector)> &);
	void _for_each_neighbor_fluid(size_t, const std::function<void(size_t, Vector)> &);
	void _for_each_neighbor_boundaries(size_t, const std::function<void(size_t, Vector, size_t)> &);
	void _resize();
	NeighborBuilder NeighborBuilder;
	Vector MaxBound;
	bool VolumeInited;

private:
	void _compute_akinci_volume();
	void _compute_density_change();
	void _compute_density_adv(real dt);
	real _compute_density_error(const real offset);
	real _divergence_solver_iteration(real dt);
	real _pressure_solve_iteration(real dt);
	void _divergence_solver_iteration_kernel(real dt);
	void _pressure_solve_iteration_kernel(real dt);
};

void test1();
}

#endif //HINAPE_DFSPH_CUDA_H
