#ifndef HINAPE_COMMON_H
#define HINAPE_COMMON_H

#include <vector>
#include <array>
#include <thrust/universal_vector.h>
#include <UT/UT_Vector3.h>

namespace HinaPE
{
using real = float;
using Vector = UT_Vector3T<real>;
using CPUVectorArray = std::vector<Vector>;
using GPUVectorArray = thrust::universal_vector<Vector>;
using CPUScalarArray = std::vector<real>;
using GPUScalarArray = thrust::universal_vector<real>;

constexpr real FluidRestDensity = 1000.f;
constexpr real eps = 1.e-5;

inline void serial_for(size_t n, const std::function<void(size_t)> &f) { for (size_t i = 0; i < n; ++i) { f(i); }}
//inline void parallel_for(size_t n, const std::function<void(size_t)> &f){ tbb::parallel_for(size_t(0), n, [&](size_t i) { f(i); }); }
inline void parallel_for(size_t n, const std::function<void(size_t)> &f) { serial_for(n, f); }

struct CubicKernel
{
	inline static void set_radius(real r)
	{
		_r = r;
		const real pi = static_cast<real>(3.14159265358979323846);

		const real h3 = _r * _r * _r;
		_k = static_cast<real>(8.0) / (pi * h3);
		_l = static_cast<real>(48.0) / (pi * h3);
		_W_0 = W(Vector{0, 0, 0});
	}

	inline static real W(const real r)
	{
		real res = 0.0;
		const real q = r / _r;
		if (q <= 1.0)
		{
			if (q <= 0.5)
			{
				const real q2 = q * q;
				const real q3 = q2 * q;
				res = _k * (static_cast<real>(6.0) * q3 - static_cast<real>(6.0) * q2 + static_cast<real>(1.0));
			} else
			{
				res = _k * (static_cast<real>(2.0) * pow(static_cast<real>(1.0) - q, static_cast<real>(3.0)));
			}
		}
		return res;
	}

	inline static real W(const Vector &r)
	{
		return W(r.length());
	}

	inline static Vector gradW(const Vector &r)
	{
		Vector res;
		const real rl = r.length();
		const real q = rl / _r;
		if ((rl > 1.0e-9) && (q <= 1.0))
		{
			Vector gradq = r / rl;
			gradq /= _r;
			if (q <= 0.5)
			{
				res = _l * q * ((real) 3.0 * q - static_cast<real>(2.0)) * gradq;
			} else
			{
				const real factor = static_cast<real>(1.0) - q;
				res = _l * (-factor * factor) * gradq;
			}
		} else
			res = Vector{0, 0, 0};

		return res;
	}

	inline static real W_zero()
	{
		return _W_0;
	}

private:
	inline static real _r;
	inline static real _k;
	inline static real _l;
	inline static real _W_0;
};

template<typename Vector3Array, typename ScalarArray, typename Vector3, typename real>
struct IFluid
{
	size_t size;
	Vector3Array x;
	Vector3Array v;
	Vector3Array f;
	ScalarArray m;
	ScalarArray V;
	ScalarArray rho;
	ScalarArray neighbor_this;
	ScalarArray neighbor_others;
};
using IFluidCPU = IFluid<CPUVectorArray, CPUScalarArray, Vector, real>;
using IFluidGPU = IFluid<GPUVectorArray, GPUScalarArray, Vector, real>;

template<typename Vector3Array, typename ScalarArray, typename Vector3, typename real>
struct AkinciBoundary
{
	size_t size;
	Vector3Array x;
	Vector3Array v;
	Vector3Array f;
	ScalarArray m;
	ScalarArray V;
	ScalarArray rho;

	ScalarArray neighbor_this;
	ScalarArray neighbor_others;
};
using AkinciBoundaryCPU = AkinciBoundary<CPUVectorArray, CPUScalarArray, Vector, real>;
using AkinciBoundaryGPU = AkinciBoundary<GPUVectorArray, GPUScalarArray, Vector, real>;
}

#endif //HINAPE_COMMON_H
