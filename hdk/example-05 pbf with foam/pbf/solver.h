#ifndef HINAPE_PBF_SOLVER_H
#define HINAPE_PBF_SOLVER_H

#include <GU/GU_Detail.h>
#include <GU/GU_NeighbourList.h>

#include <GEO/GEO_Mirror.h>

class GU_Detail;

namespace HinaPE
{

// Kernels
constexpr SYS_FORCE_INLINE fpreal WPoly6(const UT_Vector3D &pi, const UT_Vector3D &pj, fpreal radius, fpreal KPOLY);
constexpr SYS_FORCE_INLINE UT_Vector3D gradWPoly6(const UT_Vector3D &pi, const UT_Vector3D &pj, fpreal radius, fpreal KPOLY);
constexpr SYS_FORCE_INLINE UT_Vector3D WSpiky(const UT_Vector3D &pi, const UT_Vector3D &pj, fpreal radius, fpreal SPIKY);

// Geometry
void BuildEdgeGroup(GU_Detail &gdp, const UT_StringHolder &edge_name);

// Neighbor
void BuildNeighbor(const GU_Detail &gdp, GU_NeighbourList &NeighborList);

// Density
void BuildDensity(const GU_Detail &gdp, const GA_RWHandleD &DENSITY, const GU_NeighbourList &neighbor_list, const GA_ROHandleV3D &pos, const GA_ROHandleD &mass);

// Lambda
void BuildLambda(const GU_Detail &gdp, const GA_RWHandleD &LAMBDA, const GA_ROHandleD &density, const fpreal eps, const GU_NeighbourList &neighbor_list, const GA_ROHandleV3D &pos);

// DeltaP
void BuildDeltaP(const GU_Detail &gdp, const GA_RWHandleV3D &DELTA_P, const GA_ROHandleD &lambda, const GU_NeighbourList &neighbor_list, const GA_ROHandleV3D &pos);

// Collision
void ParticleCollision();

// Integrate
void IntegrateSemiEuler(GA_Offset offset, const GA_RWHandleV3D &POS, const GA_RWHandleV3D &VEL, const GA_ROHandleV3 &force, const GA_ROHandleR &inv_mass, const fpreal dt);

// Constraint
void SolveDistanceConstraint(GA_Offset p1, GA_Offset p2, const GA_RWHandleV3D &POS, const GA_ROHandleV3 &pos, const GA_ROHandleR &inv_mass, fpreal rest_length, fpreal stiffness);
}

#endif //HINAPE_PBF_SOLVER_H
