#ifndef HINAPE_PARTICLES_H
#define HINAPE_PARTICLES_H

#include <Discregrid/discregrid/include/Discregrid/All>
#include "tbb/tbb.h"

#include <functional>
#include <memory>

namespace HinaPE
{
inline void serial_for(size_t n, const std::function<void(size_t)> &f) { for (size_t i = 0; i < n; ++i) { f(i); }}
inline void parallel_for(size_t n, const std::function<void(size_t)> &f) { tbb::parallel_for(size_t(0), n, [&](size_t i) { f(i); }); }

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

template<typename real, typename Vector3, typename ScalarArray, typename Vector3Array>
struct IBenderBoundary
{
	IBenderBoundary()
	{
//		Vmap = std::make_shared<Discregrid::CubicLagrangeDiscreteGrid>();
	}
	std::shared_ptr<Discregrid::DiscreteGrid> Vmap;
};

} // namespace HinaPE

#endif //HINAPE_PARTICLES_H
