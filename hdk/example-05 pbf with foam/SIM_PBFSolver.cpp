#include "SIM_PBFSolver.h"

#include "pbf/solver.h"
#include "pbf/util_log.h"

#include <PRM/PRM_Name.h>
#include <PRM/PRM_Template.h>
#include <PRM/PRM_Shared.h>
#include <PRM/PRM_Default.h>

#include <SIM/SIM_Engine.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_Collider.h>
#include <SIM/SIM_ColliderPoint.h>
#include <SIM/SIM_ColliderBFA.h>
#include <SIM/SIM_RelationshipSource.h>
#include <SIM/SIM_PRMShared.h>
#include <SIM/SIM_GuideShared.h>
#include <SIM/SIM_Position.h>
#include <SIM/SIM_PositionSimple.h>

#include <GU/GU_Detail.h>
#include <GA/GA_Handle.h>
#include <GA/GA_Primitive.h>

#include <UT/UT_ThreadedAlgorithm.h>
#include <UT/UT_ParallelUtil.h>
#include <UT/UT_ParallelPipeline.h>

#include <SIM/SIM_Impacts.h>
#include <SIM/SIM_SweptCollisionUtility.h>
#include <SIM/SIM_SweptCollision.h>
#include <SIM/SIM_SweptCollisionData.h>
#include <SIM/SIM_SDFCollision.h>

#include <chrono>

const SIM_DopDescription *SIM_PBFSolver::GetDescription()
{
	static PRM_Name gravity("gravity", "Gravity");
	static std::array<PRM_Default, 3> gravity_default = {0, -9.8, 0};

	static PRM_Name dt("dt", "Dt");
	static PRM_Default dt_default(1e-3);

	static std::array<PRM_Template, 3> PRMS{
			PRM_Template(PRM_FLT_J, 3, &gravity, gravity_default.data()),
			PRM_Template(PRM_FLT_J, 1, &dt, &dt_default),
			PRM_Template()
	};

	static SIM_DopDescription DESC(true,
								   "pbf_solver",
								   "PBF Solver",
								   "PBFSolver",
								   classname(),
								   PRMS.data());

	static PRM_Name boundary("boundary", "Boundary");
	static std::array<PRM_Default, 3> boundary_default = {2, 2, 2};

	static std::array<PRM_Template, 4> GuidePRMS{
			PRM_Template(PRM_TOGGLE, 1, &SIMshowguideName,
						 PRMzeroDefaults),
			PRM_Template(PRM_RGB, 3,
						 &SIMcolorName, PRMoneDefaults,
						 0, &PRMunitRange),
			PRM_Template(PRM_FLT_J, 3, &boundary, boundary_default.data()),
			PRM_Template()
	};

	DESC.setGuideTemplates(GuidePRMS.data());
	return &DESC;
}

SIM_Solver::SIM_Result SIM_PBFSolver::solveSingleObjectSubclass(SIM_Engine &engine, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep, bool newobject)
{
	HinaPE::InfoLog("========== NEW FRAME ==========");

	static bool NeedReBuild = true;
	if (NeedReBuild || newobject)
	{
		// TODO: Build/Init Your Data
		// Note, the first frame is always for initialization
		NeedReBuild = false;
		return SIM_Solver::SIM_SOLVER_SUCCESS;
	}

	SIM_GeometryCopy *geo = SIM_DATA_GET(object, "Geometry", SIM_GeometryCopy);
	SIM_PositionSimple *pos = SIM_DATA_GET(object, "Position", SIM_PositionSimple);

	if (!geo || !pos)
	{
		// Do Fail Log
		HinaPE::ErrorLog<std::string>("NULLPTR - Geometry or Position is NULLPTR");
		return SIM_Solver::SIM_SOLVER_FAIL;
	}

	SIM_ObjectArray affectors;
	object.getAffectors(affectors, "SIM_RelationshipCollide");
	exint num_affectors = affectors.entries();
	for (exint i = 0; i < num_affectors; ++i)
	{
		SIM_Object &affector = *affectors(i);

		if (!affector.getName().equal(object.getName()))
		{
			SIM_Geometry *collider_geo = SIM_DATA_GET(affector, "Geometry", SIM_Geometry);
			SIM_Position *collider_pos = SIM_DATA_GET(affector, "Position", SIM_Position);

			if (!collider_geo || !collider_pos)
			{
				// Do Fail Log
				HinaPE::ErrorLog<std::string>("NULLPTR - Collider Geometry or Position is NULLPTR");
				return SIM_Solver::SIM_SOLVER_FAIL;
			}

			HinaPE::InfoLog("PASSED NEW AFFECTOR: " + affector.getName().toStdString());

			GU_ConstDetailHandle gdh = geo->getGeometry();
			GU_ConstDetailHandle collider_gdh = collider_geo->getGeometry();
			auto collider1 = HinaPE::AsFCLCollider(gdh, pos);
			auto collider2 = HinaPE::AsFCLCollider(collider_gdh, collider_pos);

			fcl::CollisionRequestf request(1, true);
			fcl::CollisionResultf result;
			std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
			fcl::collide(collider1.get(), collider2.get(), request, result);
			std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

			std::string performance_log = "Collision Time: ";
			performance_log += std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
					end - begin).count());
			performance_log += "[ms]";
			HinaPE::InfoLog(performance_log); // Debug: 47[ms] vs Release: 1[ms]

			if (result.isCollision())
			{
				result.getContacts(cached_contacts);
				HinaPE::InfoLog("Contacts Size: " + cached_contacts.size());
				for (const fcl::Contact<float> &contact: cached_contacts)
				{
					HinaPE::InfoLog(contact.b1, "info");
					HinaPE::InfoLog(contact.b2, "info");
					HinaPE::InfoLog<fcl::Vector3<float>, 3>(contact.normal, "info");
					HinaPE::InfoLog<fcl::Vector3<float>, 3>(contact.pos, "info");
					HinaPE::InfoLog(contact.penetration_depth, "info");
					HinaPE::InfoLog(contact.NONE, "info");

					UT_Vector3F current_pos = pos->getPosition();
					UT_Vector3F collision_normal = {contact.normal.x(), contact.normal.y(), contact.normal.z()};
					fpreal32 collision_depth = contact.penetration_depth;
					current_pos += collision_normal * collision_depth;
					pos->setPosition(current_pos);
				}

				result.getCostSources(cached_cost_sources);
				HinaPE::InfoLog("Contacts Size: " + cached_cost_sources.size());
				for (const fcl::CostSource<float> &cost_source: cached_cost_sources)
				{
					HinaPE::InfoLog("MY COST SOURCE");
				}
			}
		}
	}

	return SIM_Solver::SIM_SOLVER_SUCCESS;
}

SIM_Guide *SIM_PBFSolver::createGuideObjectSubclass() const
{
	// Return a shared guide so that we only have to build our geometry
	// once. But set the displayonce flag to false so that we can set
	// a different transform for each object.
	return new SIM_GuideShared(this, false);
}

void SIM_PBFSolver::buildGuideGeometrySubclass(const SIM_RootData &root, const SIM_Options &options, const GU_DetailHandle &gdh, UT_DMatrix4 *xform, const SIM_Time &t) const
{
	SIM_Data::buildGuideGeometrySubclass(root, options, gdh, xform, t);
}
