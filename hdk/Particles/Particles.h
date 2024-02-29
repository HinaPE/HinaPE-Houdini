//
// Created by LiYifan on 2024/2/26.
//

#ifndef HINAPE_HOUDINI_PARTICLES_H
#define HINAPE_HOUDINI_PARTICLES_H

#include <iostream>
#include <map>
#include <vector>
#include <UT/UT_Vector3.h>
#include <GA/GA_Types.h>

struct ParticleState
{
    GA_Offset pt_off;
    UT_Vector3 pt_pos;
};

struct Triangle {
    GA_Offset prim_off;
    UT_Vector3 prim_pos;
    ParticleState pt1,pt2,pt3;
};

class FluidParticles
{
public:
    FluidParticles() = default;
    ~FluidParticles() = default;

    void initialize(){
        positions.clear();
        velocities.clear();
        forces.clear();
        densities.clear();
        pressures.clear();

        point_neighbors.clear();
        triangle_neighbors.clear();
    }
public:
    std::map<GA_Offset,UT_Vector3> positions;
    std::map<GA_Offset,UT_Vector3> velocities;
    std::map<GA_Offset,UT_Vector3> forces;
    std::map<GA_Offset,float> densities;
    std::map<GA_Offset,float> pressures;

    // Point neighbor list
    std::map<GA_Offset,std::vector<ParticleState>> point_neighbors;
    // Triangle neighbor list
    std::map<GA_Offset,std::vector<Triangle>> triangle_neighbors;
};

using FluidParticlesPtr = std::shared_ptr<FluidParticles>;

#endif //HINAPE_HOUDINI_PARTICLES_H
