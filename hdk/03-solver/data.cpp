#include "data.h"

#include <SIM/SIM_DopDescription.h>
#include <PRM/PRM_Template.h>

#include <iostream>
#include <array>

HinaPE::real *HinaPE::ClothFactory::VERTEX_POOL = nullptr;
HinaPE::size HinaPE::ClothFactory::VERTEX_POOL_SIZE = 0;
HinaPE::size HinaPE::ClothFactory::VERTEX_POOL_USED = 0;
HinaPE::size *HinaPE::ClothFactory::INDEX_POOL = nullptr;
HinaPE::size HinaPE::ClothFactory::INDEX_POOL_SIZE = 0;
HinaPE::size HinaPE::ClothFactory::INDEX_POOL_USED = 0;

HinaClothData::HinaClothData(const SIM_DataFactory *factory) : SIM_Data(factory), SIM_OptionsUser(this) {}
HinaClothData::~HinaClothData() = default;
auto HinaClothData::GetDescription() -> const SIM_DopDescription *
{
	static std::array<PRM_Template, 1> PRMS{
			PRM_Template()
	};
	static SIM_DopDescription DESC(
			true,
			"hina_cloth_data",
			"Hina Cloth Data",
			"HinaClothData",
			classname(),
			PRMS.data()
	);
	return &DESC;
}

//void HinaPE::ClothSolver::LoadDistanceConstraint(HinaPE::size start_node, HinaPE::size end_node)
//{
////	if (start_node >= V_Num || end_node >= V_Num) // HinaPE::size is unsigned, so no need to check for negative values
////	{
////		std::cout << "Error: Invalid node index.\n";
////		return;
////	}
//
//}

void HinaPE::ClothSolver::Init(const HinaPE::ClothSolver::Args &args)
{
	ClothFactory::Init(10000, 10000);
}
void HinaPE::ClothSolver::Solve(const HinaPE::ClothSolver::Args &args)
{
	const real &dt = args.dt;
}
