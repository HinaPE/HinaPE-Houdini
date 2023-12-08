#ifndef HINAPE_PBF_DATA_H
#define HINAPE_PBF_DATA_H

#include <cmath>

namespace HinaPE
{
struct GlobalParameters
{
	fpreal PI = 3.141592653589793238462643383279502884197169399375105820974944592307816406286;
	fpreal RADIUS = 0.1;
	fpreal KPOLY = 315.0f / (64.0f * PI * std::pow(RADIUS, 9));
	fpreal SPIKY = 45.0f / (PI * pow(RADIUS, 6));
	fpreal K = 0.00001f;
	fpreal dqMag = 0.2 * RADIUS;
	fpreal wQH = KPOLY * std::pow(RADIUS * RADIUS - dqMag * dqMag, 3);
	fpreal REST_DENSITY = 1000.0f;
};

static GlobalParameters PRM;

} // HinaPE

#endif //HINAPE_PBF_DATA_H
