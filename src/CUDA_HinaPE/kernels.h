#ifndef HINAPE_KERNELS_H
#define HINAPE_KERNELS_H

#include <UT/UT_Vector.h>
#include <array>

namespace HinaPE
{
constexpr fpreal PI = 3.14159265358979323846;

template<bool use_cache = true, size_t accuracy = 10000>
struct CubicSplineKernel
{
	fpreal kernel(const fpreal r)
	{
		if (r < 0 || r >= h) return 0;

		fpreal res;
		const fpreal q = 2 * r / h;
		if constexpr (use_cache)
		{
			int cache_idx = std::floor(q * accuracy / 2.);
			res = kernel_cache[cache_idx];
		} else
		{
			fpreal b;
			if (q < 1) b = 1. - 3. / 2. * q * q + 3. / 4. * q * q * q;
			else b = 1. / 4. * (2. - q) * (2. - q) * (2. - q);
			res = a * b;
		}
		return res;
	}

	fpreal derivative(const fpreal r)
	{
		if (r < 0 || r >= h) return 0;

		fpreal res;
		const fpreal q = 2 * r / h;
		if constexpr (use_cache)
		{
			int cache_idx = std::floor(q * accuracy / 2.);
			res = derivative_cache[cache_idx];
		} else
		{
			fpreal c;
			if (q < 1) c = -3. * q + 9. / 4. * q * q;
			else c = -3. / 4. * (2. - q) * (2. - q);
			res = a * c;
		}
		return res;
	}

	UT_Vector3 gradient(const UT_Vector3 r)
	{
		const fpreal eps = 1e-6;
		fpreal r_l = r.length();
		if (r_l >= h) return UT_Vector3(0, 0, 0);
		if (r_l < eps) r_l = eps; // note: r's length can be 0!
		UT_Vector3 r_u = r / r_l;

		UT_Vector3 res;
		const fpreal q = 2 * r_l / h;
		if constexpr (use_cache)
		{
			int cache_idx = std::floor(q * accuracy / 2.);
			res = r_u * gradient_cache[cache_idx];
		} else
		{
			fpreal d;
			if (q < 1) d = (-3. * q + 9. / 4. * q * q) / h;
			else d = (-3. / 4. * (2. - q) * (2. - q)) / h;
			res = r_u * (a * d);
		}
		return res;
	}

	constexpr CubicSplineKernel(fpreal h) : h(h), a(1. / (PI * h * h * h)), kernel_cache(), derivative_cache()
	{
		if constexpr (use_cache)
		{
			for (int i = 0; i < accuracy; ++i)
			{
				const fpreal q = 2 * i / (fpreal) accuracy;
				if (q < 1)
				{
					const fpreal b = 1. - 3. / 2. * q * q + 3. / 4. * q * q * q;
					kernel_cache[i] = a * b;
					const fpreal c = -3. * q + 9. / 4. * q * q;
					derivative_cache[i] = a * c;
					const fpreal d = (-3. * q + 9. / 4. * q * q) / h;
					gradient_cache[i] = a * d;
				} else
				{
					const fpreal b = 1. / 4. * (2. - q) * (2. - q) * (2. - q);
					kernel_cache[i] = a * b;
					const fpreal c = -3. / 4. * (2. - q) * (2. - q);
					derivative_cache[i] = a * c;
					const fpreal d = (-3. / 4. * (2. - q) * (2. - q)) / h;
					gradient_cache[i] = a * d;
				}
			}
		}
	}

private:
	const fpreal h;
	const fpreal a;
	std::array<fpreal, accuracy> kernel_cache;
	std::array<fpreal, accuracy> derivative_cache;
	std::array<fpreal, accuracy> gradient_cache;
};
} // namespace HinaPE

#endif //HINAPE_KERNELS_H
