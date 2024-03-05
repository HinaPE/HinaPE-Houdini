#ifndef HINAPE_NEIGHBORS_H
#define HINAPE_NEIGHBORS_H

#include "TreeNSearch"
#include "cuNSearch.h"

namespace HinaPE
{
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

// DO NOT USE THIS ( NOT IMPLEMENTED YET )
template<typename real, typename Vector3, typename ScalarArray, typename Vector3Array>
struct NeighborBuilderCPU : INeighborBuilder<real, Vector3, ScalarArray, Vector3Array>
{
	NeighborBuilderCPU(real radius)
	{
		searcher = std::make_shared<tns::TreeNSearch>();
		searcher->set_search_radius(radius);
	}
	void init(const std::vector<Vector3Array *> &x_sets) override { for (const auto &x: x_sets) searcher->add_point_set(x->front().data(), x->size(), true, true, true); }
	void update_set(int i) override { throw std::runtime_error("Not implemented"); }
	void disable_set_to_search_from(int i) override { searcher->set_active_search(i, false /* search neighbors */, true /* BE SEARCHED by other points sets*/); }
	void resize_set(int i, Vector3Array *x) override { searcher->resize_point_set(i, x->front().data(), x->size()); }
	void build() override { searcher->run(); }
	void for_each_neighbor(size_t this_set_idx, size_t target_set_idx, size_t this_pt_idx, const std::function<void(size_t, Vector3)> &f) const override
	{
		throw std::runtime_error("Not implemented");
		auto list = searcher->get_neighborlist(this_set_idx, target_set_idx, this_pt_idx);
//		serial_for(list.size(), [&](size_t n_idx)
//		{
//			size_t j = list[n_idx];
//			Vector3 x = {target_set.GetPoints()[3 * j + 0],
//						 target_set.GetPoints()[3 * j + 1],
//						 target_set.GetPoints()[3 * j + 2]};
//			f(j, x);
//		});
	}
	size_t n_neighbors(size_t this_set_idx, size_t target_set_idx, int this_pt_idx) override
	{
		return searcher->get_neighborlist(this_set_idx, target_set_idx, this_pt_idx).size();
	}
private:
	std::shared_ptr<tns::TreeNSearch> searcher;
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
}

#endif //HINAPE_NEIGHBORS_H
