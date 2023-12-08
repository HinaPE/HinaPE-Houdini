#include "solver.h"

#include "data.h"

#include <GEO/GEO_Detail.h>

#include <iostream>
#include <cmath>

constexpr SYS_FORCE_INLINE float
HinaPE::WPoly6(const UT_Vector3F &pi, const UT_Vector3F &pj, float radius, float KPOLY)
{
	UT_Vector3F r = pi - pj;
	float rLen = r.length();
	if (rLen > radius || rLen == 0) return 0;
	return KPOLY * std::pow((radius * radius - pow(rLen, 2)), 3);
}

constexpr SYS_FORCE_INLINE UT_Vector3F
HinaPE::gradWPoly6(const UT_Vector3F &pi, const UT_Vector3F &pj, float radius, float k_poly)
{
	UT_Vector3F r = pi - pj;
	float rLen = r.length();
	if (rLen > radius || rLen == 0) { return UT_Vector3F(0.0); }

	float coeff = std::pow((radius * radius) - (rLen * rLen), 2);
	coeff *= -6 * k_poly;
	return r * coeff;
}
constexpr SYS_FORCE_INLINE UT_Vector3F
HinaPE::WSpiky(const UT_Vector3F &pi, const UT_Vector3F &pj, float radius, float spiky)
{
	UT_Vector3F r = pi - pj;
	float rLen = r.length();
	if (rLen > radius || rLen == 0) { return UT_Vector3F(0.0); }

	float coeff = (radius - rLen) * (radius - rLen);
	coeff *= spiky;
	coeff /= rLen;
	return r * -coeff;
}

void HinaPE::BuildEdgeGroup(GU_Detail &gdp, const UT_StringHolder &edge_name)
{
	std::vector<std::pair<GA_Offset, GA_Offset>> pairs;
	for (GA_Size i = 0; i < gdp.getNumPrimitives(); ++i)
	{
		GA_Offset offset = gdp.primitiveOffset(i);
		const GA_Primitive *pm = gdp.getPrimitive(offset);

		GA_Size VC = pm->getVertexCount();
		for (GA_Size j = 0; j < VC; ++j)
		{
			GA_Offset o1 = pm->getPointOffset(j);
			GA_Offset o2 = pm->getPointOffset((j + 1) % VC);
			if (o1 < o2)
				pairs.emplace_back(o1, o2);
			else
				pairs.emplace_back(o2, o1);
		}
	}
	std::sort(pairs.begin(), pairs.end());
	pairs.erase(std::unique(pairs.begin(), pairs.end(), [](const auto &a, const auto &b)
	{
		return a.first == b.first && a.second == b.second;
	}), pairs.end());

	GA_EdgeGroup *edge = gdp.createEdgeGroup(edge_name);
	for (const auto &pair: pairs)
	{
		GA_Edge e(pair.first, pair.second);
		edge->add(e);
	}
}

void HinaPE::BuildNeighbor(const GU_Detail &gdp, GU_NeighbourList &NeighborList)
{
	GU_NeighbourListParms params;
	params.setMode(GU_NeighbourListParms::UNIFORM);
	params.setOverrideRadius(true);
	params.setRadius(0.1);
	params.setRadiusScale(1.0);
	params.setRadiusAttribute("Neighbor Radius");

	NeighborList.clearLists();
	NeighborList.build(&gdp, params);
}

void HinaPE::BuildDensity(const GU_Detail &gdp, const GA_RWHandleF &DENSITY, const GU_NeighbourList &neighbor_list, const GA_ROHandleV3 &pos, const GA_ROHandleF &mass)
{
	for (GA_Size i = 0; i < gdp.getNumPoints(); ++i)
	{
		GA_Offset offset = gdp.pointOffset(i);

		UT_Vector3F pos_i = pos.get(offset);
		float mass_i = mass.get(offset);

		UT_Array<GA_Offset> ptlist;
		neighbor_list.getNeighbours(i, &gdp, ptlist);

		float density_sum = 0;
		for (const GA_Offset &neighbor_point_offset: ptlist)
		{
			UT_Vector3F pos_j = pos.get(neighbor_point_offset);
			density_sum += mass_i * WPoly6(pos_i, pos_j, PRM.RADIUS, PRM.KPOLY);
		}

		DENSITY.set(offset, density_sum);
	}
}

void HinaPE::BuildLambda(const GU_Detail &gdp, const GA_RWHandleF &LAMBDA, const GA_ROHandleF &density, float eps, const GU_NeighbourList &neighbor_list, const GA_ROHandleV3 &pos)
{
	for (GA_Size i = 0; i < gdp.getNumPoints(); ++i)
	{
		GA_Offset offset = gdp.pointOffset(i);

		UT_Vector3F pos_i = pos.get(i);
		float density_i = density.get(i);

		UT_Array<GA_Offset> ptlist;
		neighbor_list.getNeighbours(i, &gdp, ptlist);

		float densityC = density_i / PRM.REST_DENSITY - 1;
		UT_Vector3F grad_I = UT_Vector3F(0.0);
		float sum_density = 0;
		for (const GA_Offset &neighbor_point_offset: ptlist)
		{
			UT_Vector3F pos_j = pos.get(neighbor_point_offset);
			UT_Vector3F grad_J = WSpiky(pos_i, pos_j, PRM.RADIUS, PRM.SPIKY) / PRM.REST_DENSITY;
			sum_density += grad_J.length2();
			grad_I += grad_J;
		}
		sum_density += grad_I.length2();
		float lambda = -densityC / (sum_density + eps);

		LAMBDA.set(offset, lambda);
	}
}

void HinaPE::BuildDeltaP(const GU_Detail &gdp, const GA_RWHandleV3 &DELTA_P, const GA_ROHandleF &lambda, const GU_NeighbourList &neighbor_list, const GA_ROHandleV3 &pos)
{
	for (GA_Size i = 0; i < gdp.getNumPoints(); ++i)
	{
		GA_Offset offset = gdp.pointOffset(i);

		UT_Vector3F pos_i = pos.get(i);
		float lambda_i = lambda.get(i);

		UT_Array<GA_Offset> ptlist;
		neighbor_list.getNeighbours(i, &gdp, ptlist);

		UT_Vector3F delta_p = UT_Vector3F(0.0);
		for (const GA_Offset &neighbor_point_offset: ptlist)
		{
			UT_Vector3F pos_j = pos.get(neighbor_point_offset);
			float lambda_sum = lambda_i + lambda.get(neighbor_point_offset);

			float corr = WPoly6(pos_i, pos_j, PRM.RADIUS, PRM.KPOLY) / PRM.wQH;
			corr *= corr * corr * corr;
			float s_corr = -PRM.K * corr;

			delta_p += (lambda_sum + s_corr) * WSpiky(pos_i, pos_j, PRM.RADIUS, PRM.SPIKY);
		}

		delta_p /= PRM.REST_DENSITY;

		DELTA_P.set(offset, delta_p);
	}
}

void ParticleCollision(const GU_Detail &fluid, const GU_Detail &collider)
{
	GU_Detail fluid_with_collider;
	fluid_with_collider.copy(fluid);
	fluid_with_collider.copy(fluid);
	fluid_with_collider.merge(collider);
}

void HinaPE::IntegrateSemiEuler(GA_Offset offset, const GA_RWHandleV3 &pos, const GA_RWHandleV3 &vel, const GA_ROHandleV3 &force, const GA_ROHandleF &inv_mass, const float dt)
{
	auto p = pos.get(offset);
	auto v = vel.get(offset);
	auto f = force.get(offset);
	auto im = inv_mass.get(offset);

	if (im == 0) return;

	v += dt * f * im;
	p += dt * v;

	vel.set(offset, v);
	pos.set(offset, p);
}

void HinaPE::SolveDistanceConstraint(GA_Offset p1, GA_Offset p2, const GA_RWHandleV3 &POS, const GA_ROHandleV3 &pos, const GA_ROHandleF &inv_mass, float rest_length, float stiffness)
{
	auto pos1 = pos.get(p1);
	auto pos2 = pos.get(p2);
	auto im1 = inv_mass.get(p1);
	auto im2 = inv_mass.get(p2);

	if (im1 == 0 && im2 == 0) return;

	auto C = (pos1 - pos2).length() - rest_length;
	auto dC = (pos1 - pos2);
	dC.normalize();

	auto dP1 = -stiffness * im1 / (im1 + im2) * C * dC;
	auto dP2 = +stiffness * im2 / (im1 + im2) * C * dC;

	pos1 += dP1;
	pos2 += dP2;

	POS.set(p1, pos1);
	POS.set(p2, pos2);
}
