#include "CUDA_HinaPE/kernels.h"
#include <iostream>
int main()
{
	HinaPE::CubicSplineKernel<true, 10000> kernel_cache(0.036);
	HinaPE::CubicSplineKernel<false> kernel_computed(0.036);
	for (fpreal v = 0; v <= 0.036; v += 0.0001)
		std::cout << " " << kernel_cache.kernel(v) - kernel_computed.kernel(v);
	std::cout << std::endl;
	for (fpreal v = 0; v <= 0.036; v += 0.0001)
		std::cout << " " << kernel_cache.derivative(v) - kernel_computed.derivative(v);
	std::cout << std::endl;
	for (fpreal v = 0; v <= 0.036; v += 0.0001)
		std::cout << " "
				  << kernel_cache.gradient(v * UT_Vector3(0, 1, 0)).y() - kernel_computed.gradient(v * UT_Vector3(0, 1, 0)).y();
}
