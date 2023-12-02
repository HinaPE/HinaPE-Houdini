#include "solver.h"

#include <PRM/PRM_Name.h>
#include <PRM/PRM_Template.h>
#include <PRM/PRM_Shared.h>
#include <PRM/PRM_Default.h>

#include <SIM/SIM_Engine.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_GeometryCopy.h>

#include <GU/GU_Detail.h>
#include <GA/GA_Handle.h>
#include <GA/GA_Primitive.h>

#include <UT/UT_ThreadedAlgorithm.h>
#include <UT/UT_ParallelUtil.h>
#include <UT/UT_ParallelPipeline.h>

#include <array>
#include <iostream>

UT_StringHolder EdgeClothGroupName("EdgesForCloth");
UT_StringHolder IsFixAttributeName("IsFix");
UT_StringHolder PredictPositionAttributeName("PdP");

std::vector<PBDClothSolver::DistanceConstraint> PBDClothSolver::DCs;

const SIM_DopDescription *PBDClothSolver::GetDescription()
{
	static PRM_Name gravity("gravity", "Gravity");
	static std::array<PRM_Default, 3> gravity_default = {0, -9.8, 0};

	static PRM_Name stiffness("stiffness", "Stiffness");
	static PRM_Default stiffness_default(1);

	static PRM_Name constraint_iteration("constraint_iteration", "Constraint Iteration");
	static PRM_Default constraint_iteration_default(3);

	static std::array<PRM_Template, 4> PRMS{
			PRM_Template(PRM_FLT_J, 3, &gravity, gravity_default.data()),
			PRM_Template(PRM_FLT_J, 1, &stiffness, &stiffness_default),
			PRM_Template(PRM_INT_J, 1, &constraint_iteration, &constraint_iteration_default),
			PRM_Template()
	};

	static SIM_DopDescription DESC(true,
								   "pbd_cloth_solver",
								   "PBD Cloth Solver",
								   "PBDClothSolver",
								   classname(),
								   PRMS.data());
	return &DESC;
}

SIM_Solver::SIM_Result PBDClothSolver::solveSingleObjectSubclass(SIM_Engine &, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep, bool newobject)
{
	Log.reset();

	static bool NeedReBuild = true;

	if (NeedReBuild || newobject)
	{
		init(object);
		NeedReBuild = false;
	} else
	{
		solve(object, timestep);
	}

	return Log.report();
}

#include <vector>
#include <utility>
#include <algorithm>
void PBDClothSolver::init(SIM_Object &obj) const
{
	SIM_GeometryCopy *geo;
	geo = SIM_DATA_CREATE(obj, "Geometry", SIM_GeometryCopy,
						  SIM_DATA_RETURN_EXISTING | SIM_DATA_ADOPT_EXISTING_ON_DELETE);
	if (!geo)
		Log.error_nullptr("INIT::SIM_GeometryCopy");

	{
		SIM_GeometryAutoWriteLock lock(geo);
		GU_Detail &gdp = lock.getGdp();

		{
			GA_RWAttributeRef velref = gdp.addFloatTuple(GA_ATTRIB_POINT, gdp.getStdAttributeName(GEO_ATTRIBUTE_VELOCITY), 3, GA_Defaults(0));
			velref.setTypeInfo(GA_TYPE_VECTOR);
			GA_RWAttributeRef predictposref = gdp.addFloatTuple(GA_ATTRIB_POINT, PredictPositionAttributeName, 3, GA_Defaults(0));
			predictposref.setTypeInfo(GA_TYPE_VECTOR);
		}

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

			GA_EdgeGroup *edge = gdp.createEdgeGroup(EdgeClothGroupName);
			DCs.clear();
			for (const auto &pair: pairs)
			{
				GA_Edge e(pair.first, pair.second);
				edge->add(e);

				fpreal rest_length = (gdp.getPos3(pair.first) - gdp.getPos3(pair.second)).length();
				DistanceConstraint dc{pair.first, pair.second, rest_length};
				DCs.emplace_back(dc);
			}
		}
	}

	std::cout << "Inited" << '\n';
}

void PBDClothSolver::solve(SIM_Object &obj, const SIM_Time &dt) const
{
	SIM_GeometryCopy *geo;
	geo = SIM_DATA_GET(obj, "Geometry", SIM_GeometryCopy);
	if (!geo)
	{
		Log.error_nullptr("SOLVE::SIM_GeometryCopy");
		return;
	}

	// prepare all parameters
	UT_Vector3 gravity = getGravity();
	int64 constraint_iteration = getConstraintIteration();
	fpreal stiffness = getStiffness();

	{
		SIM_GeometryAutoWriteLock lock(geo);
		GU_Detail &gdp = lock.getGdp();

		fpreal mass = 1; // assume mass is 1

		GA_ROHandleI isfixhandle = gdp.findPointAttribute(IsFixAttributeName);
		GA_RWHandleV3 velhandle = gdp.findPointAttribute(gdp.getStdAttributeName(GEO_ATTRIBUTE_VELOCITY));
		GA_RWHandleV3 predictposhandle = gdp.findPointAttribute(PredictPositionAttributeName);

		// predict position
		for (GA_Size i = 0; i < gdp.getNumPoints(); ++i)
		{
			GA_Offset offset = gdp.pointOffset(i);

			UT_Vector3 external_force = gravity; // TODO: extend external force
			UT_Vector3 velocity = velhandle.get(offset);
			UT_Vector3 position = gdp.getPos3(offset);

			if (isfixhandle.get(offset))
				velocity = UT_Vector3(0.);
			else
				velocity = velocity + dt * external_force;
			position = position + dt * velocity;

			velhandle.set(offset, velocity);
			predictposhandle.set(offset, position);
		}

		// solve constraints
		while (constraint_iteration--)
		{
			for (const auto &DC: DCs)
			{
				UT_Vector3 Pos1 = predictposhandle.get(DC.P1);
				UT_Vector3 Pos2 = predictposhandle.get(DC.P2);
				fpreal RL = DC.rest_length;

				fpreal C = (Pos1 - Pos2).length() - RL;
				UT_Vector3 dC = (Pos1 - Pos2);
				dC.normalize();

				UT_Vector3 dP1 = -stiffness * mass / (mass + mass) * C * dC;
				UT_Vector3 dP2 = +stiffness * mass / (mass + mass) * C * dC;

				if (isfixhandle.get(DC.P1))
				{
					dP1 = 0;
					dP2 *= 2;
				}
				if (isfixhandle.get(DC.P2))
				{
					dP2 = 0;
					dP1 *= 2;
				}

				Pos1 += dP1;
				Pos2 += dP2;

				predictposhandle.set(DC.P1, Pos1);
				predictposhandle.set(DC.P2, Pos2);
			}
		}

		// write back position
		for (GA_Size i = 0; i < gdp.getNumPoints(); ++i)
		{
			GA_Offset offset = gdp.pointOffset(i);

			UT_Vector3 predictpos = predictposhandle.get(offset);
			UT_Vector3 originpos = gdp.getPos3(offset);

			UT_Vector3 vel = (predictpos - originpos) / dt;

			velhandle.set(offset, vel);
			if (!isfixhandle.get(offset))
				gdp.setPos3(offset, predictpos);
		}
	}

	std::cout << "Solved" << '\n';
}
