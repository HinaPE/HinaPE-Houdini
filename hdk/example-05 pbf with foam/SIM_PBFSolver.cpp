#include "SIM_PBFSolver.h"

#include "pbf/solver.h"

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
#include <SIM/SIM_RelationshipSource.h>
#include <SIM/SIM_PRMShared.h>
#include <SIM/SIM_GuideShared.h>

#include <GU/GU_Detail.h>
#include <GA/GA_Handle.h>
#include <GA/GA_Primitive.h>

#include <UT/UT_ThreadedAlgorithm.h>
#include <UT/UT_ParallelUtil.h>
#include <UT/UT_ParallelPipeline.h>

#include <array>
#include <iostream>

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

SIM_Solver::SIM_Result SIM_PBFSolver::solveSingleObjectSubclass(SIM_Engine &, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep, bool newobject)
{
//	static bool NeedReBuild = true;
//
//	if (NeedReBuild || newobject)
//	{
////		init(object);
//		NeedReBuild = false;
//	} else
//	{
////		solve(object, timestep);
//	}

	SIM_GeometryCopy *geo = SIM_DATA_GET(object, "Geometry", SIM_GeometryCopy);
	SIM_GeometryAutoWriteLock lock(geo);
	GU_Detail &gdp = lock.getGdp();

	std::cout << "obj points offset: ";
	for (GA_Size i = 0; i < gdp.getNumPoints(); ++i)
	{
		GA_Offset offset = gdp.pointOffset(i);
		std::cout << offset << " ";
	}
	std::cout << "\n";

	// After Effect
	SIM_ObjectArray affectors;
	object.getAffectors(affectors, "SIM_RelationshipCollide");
	for (GA_Size i = 0; i < affectors.entries(); ++i)
	{
		SIM_Object &affector = *affectors(i);
		if (!affector.getName().equal(object.getName()))
		{
			SIM_Geometry *collider_geo = SIM_DATA_GET(affector, "Geometry", SIM_Geometry);
			SIM_GeometryAutoReadLock collider_lock(collider_geo);
			const GU_Detail *collider_gdp = collider_lock.getGdp();

			std::cout << "collider points offset: ";
			for (GA_Size i = 0; i < collider_gdp->getNumPoints(); ++i)
			{
				GA_Offset offset = collider_gdp->pointOffset(i);
				std::cout << offset << " ";
			}
			std::cout << "\n";

			GU_Detail *merged_gdp = new GU_Detail();
			merged_gdp->merge(gdp); // Track offsets By yourself
			merged_gdp->merge(*collider_gdp);

			std::cout << "merged points offset: ";
			for (GA_Size i = 0; i < merged_gdp->getNumPoints(); ++i)
			{
				GA_Offset offset = merged_gdp->pointOffset(i);
				std::cout << offset << " ";
			}
			std::cout << "\n";
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
