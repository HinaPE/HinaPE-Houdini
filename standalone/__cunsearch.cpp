#ifdef WIN32
#include "cuNSearch.h"
#endif

#include <vector>
#include <array>

int main()
{
	float radius = 1.1;
#ifdef WIN32
	cuNSearch::NeighborhoodSearch nsearch(radius);

	std::vector<std::array<cuNSearch::Real, 3>> pos_1;
	pos_1.emplace_back(std::array<cuNSearch::Real, 3>{0., 0., 0.});
	pos_1.emplace_back(std::array<cuNSearch::Real, 3>{1., 0., 0.});
	pos_1.emplace_back(std::array<cuNSearch::Real, 3>{2., 0., 0.});
	pos_1.emplace_back(std::array<cuNSearch::Real, 3>{3., 0., 0.});
	pos_1.emplace_back(std::array<cuNSearch::Real, 3>{4., 0., 0.});
	pos_1.emplace_back(std::array<cuNSearch::Real, 3>{5., 0., 0.});
	unsigned int ps_1 = nsearch.add_point_set(pos_1.front().data(), pos_1.size());

//	std::vector<std::array<cuNSearch::Real, 3>> pos_2;
//	pos_2.emplace_back(std::array<cuNSearch::Real, 3>{0., 0., 1.});
//	pos_2.emplace_back(std::array<cuNSearch::Real, 3>{1., 0., 1.});
//	pos_2.emplace_back(std::array<cuNSearch::Real, 3>{2., 0., 1.});
//	pos_2.emplace_back(std::array<cuNSearch::Real, 3>{3., 0., 1.});
//	pos_2.emplace_back(std::array<cuNSearch::Real, 3>{4., 0., 1.});
//	pos_2.emplace_back(std::array<cuNSearch::Real, 3>{5., 0., 1.});
//	unsigned int ps_2 = nsearch.add_point_set(pos_2.front().data(), pos_2.size());
//
//	std::vector<std::array<cuNSearch::Real, 3>> pos_3;
//	pos_3.emplace_back(std::array<cuNSearch::Real, 3>{0., 0., 2.});
//	pos_3.emplace_back(std::array<cuNSearch::Real, 3>{1., 0., 2.});
//	pos_3.emplace_back(std::array<cuNSearch::Real, 3>{2., 0., 2.});
//	pos_3.emplace_back(std::array<cuNSearch::Real, 3>{3., 0., 2.});
//	pos_3.emplace_back(std::array<cuNSearch::Real, 3>{4., 0., 2.});
//	pos_3.emplace_back(std::array<cuNSearch::Real, 3>{5., 0., 2.});
//	unsigned int ps_3 = nsearch.add_point_set(pos_3.front().data(), pos_3.size());

	nsearch.set_active(true);

	nsearch.find_neighbors();

	auto &ps1_set = nsearch.point_set(ps_1);
//	auto &ps2_set = nsearch.point_set(ps_2);
//	auto &ps3_set = nsearch.point_set(ps_3);

	for (int i = 0; i <= 5; ++i)
		std::cout << "1." << i << ": " << ps1_set.n_neighbors(ps_1, i) << std::endl;

//	std::cout << std::endl;
//
//	for (int i = 0; i <= 5; ++i)
//		std::cout << "2." << i << ": " << ps2_set.n_neighbors(ps_2, i) << std::endl;
//
//	std::cout << std::endl;
//
//	for (int i = 0; i <= 5; ++i)
//		std::cout << "3." << i << ": " << ps2_set.n_neighbors(ps_3, i) << std::endl;
#endif
	return 0;
}
