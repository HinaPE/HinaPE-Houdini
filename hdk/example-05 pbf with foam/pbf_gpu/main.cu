#include <iostream>

#include <cuda_runtime.h>
#include <thrust/device_vector.h>

__global__ void cuda_hello()
{
	unsigned int tid = blockDim.x * blockIdx.x + threadIdx.x;
	unsigned int tnum = gridDim.x * blockDim.x;
	printf("Hello World from GPU! %d, %d, %d | %d, %d, %d | %d, %d, %d | %d, %d, %d\n", threadIdx.x, threadIdx.y, threadIdx.z, gridDim.x, gridDim.y, gridDim.z, blockIdx.x, blockIdx.y, blockIdx.z, blockDim.x, blockDim.y, blockDim.z);
}

int main()
{
	cuda_hello<<<3, 3>>>(); // <<< grid_dim， block_dim >>> // block num, thread num
	cudaDeviceSynchronize();
	return 0;
}
