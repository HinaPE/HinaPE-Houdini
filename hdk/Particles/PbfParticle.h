//
// Created by LiYifan on 2024/2/26.
//

#ifndef HINAPE_HOUDINI_PBFPARTICLE_H
#define HINAPE_HOUDINI_PBFPARTICLE_H

#include "Particles.h"

#include <iostream>
#include <map>

class PbfParticle : public FluidParticles
{
public:
    PbfParticle() = default;
    ~PbfParticle() = default;
public:
    std::map<GA_Offset,UT_Vector3> predicted_positions;
};

using PbfParticlePtr = std::shared_ptr<PbfParticle>;

#endif //HINAPE_HOUDINI_PBFPARTICLE_H
