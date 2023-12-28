#ifndef SIM_PBF_DATA_H
#define SIM_PBF_DATA_H

#include <vector>

namespace HinaPE_PBF
{
using real = float;
using size = int32_t;

struct PBF_DATA
{
	// Sim Data
	std::vector<real> positions;
	std::vector<real> positions_predicted;
	std::vector<real> velocities;
	std::vector<real> forces;
	std::vector<real> mass;
	std::vector<real> inv_mass;

	// State Data
	std::vector<real> densities;
	std::vector<real> lambdas;
	std::vector<real> delta_p;
};
}

#endif //SIM_PBF_DATA_H
