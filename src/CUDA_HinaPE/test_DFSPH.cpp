#include <DFSPH.h>
#include <iostream>

int main()
{
	HinaPE::DFSPHSolverCPU solver(0.02f);
	HinaPE::parallel_for(100, [&](size_t i)
	{
		std::cout << i << std::endl;
	});
	return 0;
}
