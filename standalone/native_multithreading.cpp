#include <UT/UT_ThreadedAlgorithm.h>
#include <iostream>
class NativeMultiThreading
{
public:
	THREADED_METHOD(GAS_Hina_DFSPHSolver, true, _test)
	void _testPartial(const UT_JobInfo &info)
	{
		int i,n;
		for (info.divideWork(10000, i, n); i < n; ++i)
		{
			std::cout << i << std::endl;
			std::cout << "Job: " << info.job() << std::endl;
			std::cout << "NumJobs: " << info.numJobs() << std::endl;
		}
	}
};

int main()
{
	NativeMultiThreading t;
	t._test();
	return 0;
}
