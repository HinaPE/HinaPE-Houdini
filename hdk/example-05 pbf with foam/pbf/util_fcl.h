#ifndef HINAPE_UTIL_FCL_H
#define HINAPE_UTIL_FCL_H

#include "fcl/fcl.h"
#include <GU/GU_Detail.h>
#include <GU/GU_PrimPoly.h>
#include <GEO/GEO_Macros.h>
#include <SIM/SIM_Position.h>

// FCL: https://github.com/flexible-collision-library/fcl
// CCD: https://github.com/danfis/libccd

namespace HinaPE
{
// Note: Make sure Primitive is Triangle
std::shared_ptr<fcl::CollisionObjectf> AsFCLCollider(const GU_Detail &gdp, const SIM_Position &position);
std::shared_ptr<fcl::CollisionObjectf> AsFCLCollider(const GU_Detail *gdp, const SIM_Position *position);
std::shared_ptr<fcl::CollisionObjectf> AsFCLCollider(const GU_ConstDetailHandle gdh, const SIM_Position *position);
}

#endif //HINAPE_UTIL_FCL_H
