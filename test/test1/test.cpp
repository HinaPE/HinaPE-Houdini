#include "src/utils.h"

#include <random>
#include <iostream>

bool Test_NativeArrayHandle()
{
	std::vector<real> test;
	for (int i = 0; i < 3000; ++i)
		test.emplace_back(i);

	NativeArrayHandle test_handle(&test);
	std::cout << test_handle.geti(0) << std::endl;
	std::cout << test_handle.geti(1) << std::endl;
	std::cout << test_handle.geti(2) << std::endl;
	std::cout << test_handle.geti(3) << std::endl;
	std::cout << test_handle.geti(4) << std::endl;

	return true;
}

bool Test_GA_ArrayHandle()
{
	// TODO: Implement
	return true;
}

bool Test_MultipleUT_Vector3TArrayHandle()
{
	std::vector<real> test1, test2, test3;
	for (int i = 0; i < 3000; ++i)
		test1.emplace_back(i);
	for (int i = 0; i < 3000; ++i)
		test2.emplace_back(i + 3000);
	for (int i = 0; i < 3000; ++i)
		test3.emplace_back(i + 6000);
	NativeArrayHandle test_handle1(&test1);
	NativeArrayHandle test_handle2(&test2);
	NativeArrayHandle test_handle3(&test3);

	std::vector<std::pair<UT_Vector3TArrayHandle<std::vector<real>> *, sz>> test_array;
	test_array.emplace_back(&test_handle1, 3000);
	test_array.emplace_back(&test_handle2, 3000);
	test_array.emplace_back(&test_handle3, 3000);

	MultipleUT_Vector3TArrayHandle<std::vector<real>> test_handle(&test_array);
	std::cout << test_handle.geti(0) << std::endl;
	std::cout << test_handle.geti(1) << std::endl;
	std::cout << test_handle.geti(2) << std::endl;
	std::cout << test_handle.geti(3000) << std::endl;
	std::cout << test_handle.geti(3001) << std::endl;
	std::cout << test_handle.geti(3002) << std::endl;
	std::cout << test_handle.geti(6000) << std::endl;
	std::cout << test_handle.geti(6001) << std::endl;
	std::cout << test_handle.geti(6002) << std::endl;
	return true;
};

int main()
{
	Test_NativeArrayHandle();
	Test_MultipleUT_Vector3TArrayHandle();
	return 0;
}
