#ifndef HINAPE_PBF_SOLVER_H
#define HINAPE_PBF_SOLVER_H

#include <GU/GU_Detail.h>
#include <GU/GU_NeighbourList.h>

#include <GEO/GEO_Mirror.h>

class GU_Detail;

namespace HinaPE
{

// Kernels
constexpr SYS_FORCE_INLINE float WPoly6(const UT_Vector3F &pi, const UT_Vector3F &pj, float radius, float KPOLY);
constexpr SYS_FORCE_INLINE UT_Vector3F gradWPoly6(const UT_Vector3F &pi, const UT_Vector3F &pj, float radius, float KPOLY);
constexpr SYS_FORCE_INLINE UT_Vector3F WSpiky(const UT_Vector3F &pi, const UT_Vector3F &pj, float radius, float SPIKY);

// Geometry
void BuildEdgeGroup(GU_Detail &gdp, const UT_StringHolder &edge_name);

// Neighbor
void BuildNeighbor(const GU_Detail &gdp, GU_NeighbourList &NeighborList);

// Density
void BuildDensity(const GU_Detail &gdp, const GA_RWHandleF &DENSITY, const GU_NeighbourList &neighbor_list, const GA_ROHandleV3 &pos, const GA_ROHandleF &mass);

// Lambda
void BuildLambda(const GU_Detail &gdp, const GA_RWHandleF &LAMBDA, const GA_ROHandleF &density, float eps, const GU_NeighbourList &neighbor_list, const GA_ROHandleV3 &pos);

// DeltaP
void BuildDeltaP(const GU_Detail &gdp, const GA_RWHandleV3 &DELTA_P, const GA_ROHandleF &lambda, const GU_NeighbourList &neighbor_list, const GA_ROHandleV3 &pos);

// Collision
void ParticleCollision();

// Integrate
void IntegrateSemiEuler(GA_Offset offset, const GA_RWHandleV3 &POS, const GA_RWHandleV3 &VEL, const GA_ROHandleV3 &force, const GA_ROHandleF &inv_mass, const float dt);

// Constraint
void SolveDistanceConstraint(GA_Offset p1, GA_Offset p2, const GA_RWHandleV3 &POS, const GA_ROHandleV3 &pos, const GA_ROHandleF &inv_mass, float rest_length, float stiffness);
}

#endif //HINAPE_PBF_SOLVER_H
