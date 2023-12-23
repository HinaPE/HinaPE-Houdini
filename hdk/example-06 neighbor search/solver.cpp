#include "solver.h"

#include <PRM/PRM_Name.h>
#include <PRM/PRM_Template.h>
#include <PRM/PRM_Shared.h>
#include <PRM/PRM_Default.h>

#include <SIM/SIM_Engine.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_PositionSimple.h>
#include <SIM/SIM_GeometryCopy.h>

#include <GU/GU_Detail.h>
#include <GA/GA_Handle.h>

#include <filesystem>
#include <iostream>

const SIM_DopDescription *NeighborSearchSolver::GetDescription()
{
	static PRM_Name neighbor_radius("neighbor_radius", "Neighbor Radius");
	static PRM_Default neighbor_radius_default = 0.5f;

	static std::array<PRM_Template, 2> PRMS{
			PRM_Template(PRM_FLT_J, 1, &neighbor_radius, &neighbor_radius_default),
			PRM_Template()
	};

	static SIM_DopDescription DESC(true,
								   "neighbor_search_solver",
								   "Neighbor Search Solver",
								   "NeighborSearchSolver",
								   classname(),
								   PRMS.data());
	return &DESC;
}

#include "TreeNSearch.h"
SIM_Solver::SIM_Result NeighborSearchSolver::solveSingleObjectSubclass(SIM_Engine &, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep, bool newobject)
{
	static bool NeedReBuild = true;

	SIM_ObjectArray affectors;
	object.getAffectors(affectors, "SIM_RelationshipCollide");
	exint num_affectors = affectors.entries();

	SIM_GeometryCopy *geo_this = SIM_DATA_GET(object, "Geometry", SIM_GeometryCopy);

	if (NeedReBuild || newobject)
	{
		SIM_GeometryAutoWriteLock lock(geo_this);
		GU_Detail &gdp = lock.getGdp();
		GA_RWAttributeRef neighbor_sum_ref = gdp.addIntTuple(GA_ATTRIB_POINT, "Neighbors", 1);
		neighbor_sum_ref.setTypeInfo(GA_TYPE_VOID);
		NeedReBuild = false;
		return SIM_SOLVER_SUCCESS;
	}

	for (exint i = 0; i < num_affectors; ++i)
	{
		SIM_Object &affector = *affectors(i);

		if (!affector.getName().equal(object.getName()))
		{
			SIM_Geometry *geo_affector = SIM_DATA_GET(object, "Geometry", SIM_Geometry);

			if (!geo_this || !geo_affector)
				return SIM_Solver::SIM_SOLVER_FAIL;

			const GU_Detail *gdp_this = geo_this->getGeometry().readLock();
			const GU_Detail *gdp_affector = geo_affector->getGeometry().readLock();

			float search_radius = getNeighborRadius();
			std::vector<std::array<float, 3>> points_this, points_affector;
			points_this.reserve(gdp_this->getNumPoints());
			points_affector.reserve(gdp_affector->getNumPoints());
			tns::TreeNSearch nsearch;
			nsearch.set_search_radius(search_radius);

			{
				GA_Offset ptoff;
				GA_ROHandleV3 P_h(gdp_this->getP());
				GA_FOR_ALL_PTOFF(gdp_this, ptoff)
					{
						UT_Vector3 P = P_h(ptoff);
						points_this.emplace_back(std::array<float, 3>{P.x(), P.y(), P.z()});
					}
			}

			{
				GA_Offset ptoff;
				GA_ROHandleV3 P_h(gdp_affector->getP());
				GA_FOR_ALL_PTOFF(gdp_affector, ptoff)
					{
						UT_Vector3 P = P_h(ptoff);
						points_affector.emplace_back(std::array<float, 3>{P.x(), P.y(), P.z()});
					}
			}

			const int set_this = nsearch.add_point_set(points_this[0].data(), points_this.size());
			const int set_affector = nsearch.add_point_set(points_affector[0].data(), points_affector.size());
			nsearch.set_active_search(set_this, set_affector);
			nsearch.run();

			{
				GA_Offset ptoff;
				GA_ROHandleV3 P_h(gdp_this->getP());
				GA_FOR_ALL_PTOFF(gdp_this, ptoff)
					{
						GA_Index ptidx = gdp_this->pointIndex(ptoff);
						const tns::NeighborList neighborlist_this_affector = nsearch.get_neighborlist(set_this, set_affector, ptidx);
						neighborlist_this_affector.size();

						SIM_GeometryAutoWriteLock lock(geo_this);
						GU_Detail &gdp = lock.getGdp();
						GA_RWHandleI neighbor_sum_handle = gdp.findPointAttribute("Neighbors");
						neighbor_sum_handle.set(ptoff, neighborlist_this_affector.size());
					}
			}
		}
	}

	return SIM_SOLVER_SUCCESS;
}
