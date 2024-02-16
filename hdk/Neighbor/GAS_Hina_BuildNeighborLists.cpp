#include "GAS_Hina_BuildNeighborLists.h"
#include <Particles/SIM_Hina_Particles.h>
#include <Particles/SIM_Hina_Akinci2012BoundaryParticles.h>
#include <CUDA_CubbyFlow/Core/Searcher/PointParallelHashGridSearcher.hpp>

#ifdef WIN32
#include "cuNSearch.h"
#endif

#ifdef WIN32
GAS_HINA_SUBSOLVER_IMPLEMENT(
		BuildNeighborLists,
		true,
		false,
		TARGET_PARTICLE_GEOMETRY(SIM_Hina_Particles)
				HINA_FLOAT_PARAMETER(KernelRadius, 0.036)

				static std::array<PRM_Name, 3> Backend = {\
            PRM_Name("0", "TBBParallelHash"), \
            PRM_Name("1", "CUDA"), \
            PRM_Name(nullptr), \
}; \
        static PRM_Name BackendName("Backend", "Backend"); \
        static PRM_Default BackendNameDefault(0, "TBBParallelHash"); \
        static PRM_ChoiceList CL(PRM_CHOICELIST_SINGLE, Backend.data()); \
        PRMS.emplace_back(PRM_ORD, 1, &BackendName, &BackendNameDefault, &CL); \
)
#else
GAS_HINA_SUBSOLVER_IMPLEMENT(
		BuildNeighborLists,
		true,
		false,
		TARGET_PARTICLE_GEOMETRY(SIM_Hina_Particles)
		HINA_FLOAT_PARAMETER(KernelRadius, 0.036)

		static std::array<PRM_Name, 3> Backend = {\
			PRM_Name("0", "TBBParallelHash"), \
			PRM_Name(nullptr), \
		}; \
		static PRM_Name BackendName("Backend", "Backend"); \
		static PRM_Default BackendNameDefault(0, "TBBParallelHash"); \
		static PRM_ChoiceList CL(PRM_CHOICELIST_SINGLE, Backend.data()); \
		PRMS.emplace_back(PRM_ORD, 1, &BackendName, &BackendNameDefault, &CL); \
)
#endif

void GAS_Hina_BuildNeighborLists::_init() {}
void GAS_Hina_BuildNeighborLists::_makeEqual(const GAS_Hina_BuildNeighborLists *src) {}
bool GAS_Hina_BuildNeighborLists::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	std::map<UT_String, std::vector<std::array<cuNSearch::Real, 3>>> temp_positions;

	// ========== 1. Fetch Fluid Particles ==========
	SIM_Hina_Particles *fluid_particles = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_Particles);
	CHECK_NULL_RETURN_BOOL(fluid_particles)
	double spacing = fluid_particles->getTargetSpacing();
	double radius = getKernelRadius();
	fluid_particles->neighbor_lists_cache.clear();
	fluid_particles->other_neighbor_lists_cache.clear();
	UT_String fluid_obj_name = obj->getName();
	temp_positions.try_emplace(fluid_obj_name);


	// ========== 2. Fetch Boundaries Particles ==========
	std::map<UT_String, SIM_Hina_Akinci2012BoundaryParticles *> boundaries_akinci;
	{
		SIM_ObjectArray affectors;
		obj->getAffectors(affectors, "SIM_RelationshipCollide");
		exint num_affectors = affectors.entries();
		for (int i = 0; i < num_affectors; ++i)
		{
			SIM_Object *obj_collider = affectors(i);
			if (obj_collider->getName().equal(obj->getName()))
				continue;

			UT_String boundary_obj_name = obj_collider->getName();
			temp_positions.try_emplace(boundary_obj_name);
			SIM_Hina_Akinci2012BoundaryParticles *boundary_akinci = SIM_DATA_GET(*obj_collider, SIM_Hina_Akinci2012BoundaryParticles::DATANAME, SIM_Hina_Akinci2012BoundaryParticles);
			if (boundary_akinci)
			{
				boundaries_akinci[boundary_obj_name] = boundary_akinci;
				boundary_akinci->neighbor_lists_cache.clear();
				boundary_akinci->other_neighbor_lists_cache.clear();
				boundary_akinci->UpdateBoundaryParticles(obj_collider);
			}
		}
	}

	// ========== 3. Build Neighbor Lists ==========
	if (getBackend() == 1) // CUDA
	{
#ifdef WIN32
		cuNSearch::NeighborhoodSearch nsearch(radius);

		// add fluid particles
		unsigned int fluid_point_set_index;
		{
			std::vector<std::array<cuNSearch::Real, 3>> &positions = temp_positions[fluid_obj_name];
			SIM_GeometryAutoWriteLock lock(fluid_particles);
			GU_Detail &gdp = lock.getGdp();
			positions.resize(gdp.getNumPoints());
			{
				GA_Offset pt_off;
				GA_FOR_ALL_PTOFF(&gdp, pt_off)
					{
						GA_Index pt_idx = gdp.pointIndex(pt_off);
						UT_Vector3 pos = gdp.getPos3(pt_off);
						positions[pt_idx][0] = pos.x();
						positions[pt_idx][1] = pos.y();
						positions[pt_idx][2] = pos.z();
					}
			}
			fluid_point_set_index = nsearch.add_point_set(positions.front().data(), positions.size(), true, true);
		}

		// add boundaries particles
		std::map<UT_String, unsigned int> boundary_point_set_indices;
		{
			for (auto &pair: boundaries_akinci)
			{
				UT_String boundary_obj_name = pair.first;
				SIM_Hina_Akinci2012BoundaryParticles *boundary_particles = pair.second;
				std::vector<std::array<cuNSearch::Real, 3>> &positions = temp_positions[boundary_obj_name];
				SIM_GeometryAutoWriteLock lock(boundary_particles);
				GU_Detail &gdp = lock.getGdp();
				positions.resize(gdp.getNumPoints());
				{
					GA_Offset pt_off;
					GA_FOR_ALL_PTOFF(&gdp, pt_off)
						{
							GA_Index pt_idx = gdp.pointIndex(pt_off);
							UT_Vector3 pos = gdp.getPos3(pt_off);
							positions[pt_idx][0] = pos.x();
							positions[pt_idx][1] = pos.y();
							positions[pt_idx][2] = pos.z();
						}
				}
				boundary_point_set_indices[boundary_obj_name] = nsearch.add_point_set(positions.front().data(), positions.size(), false /* NOT dynamic */, false /* NOT search neighbors */, true /* BE SEARCHED by other points sets*/);
			}
		}

		// build neighbors
		nsearch.find_neighbors();


		// ========== 4. Set Neighbor Lists ==========
		{
			auto &fluid_point_set = nsearch.point_set(fluid_point_set_index);
			SIM_GeometryAutoWriteLock lock(fluid_particles);
			GU_Detail &gdp = lock.getGdp();
			GA_Offset pt_off;
			GA_FOR_ALL_PTOFF(&gdp, pt_off)
				{
					GA_Index pt_idx = gdp.pointIndex(pt_off);

					// self neighbors
					{
						auto neighbor_count = fluid_point_set.n_neighbors(fluid_point_set_index, pt_idx);
						for (int nidx = 0; nidx < neighbor_count; ++nidx)
						{
							auto fn_idx = fluid_point_set.neighbor(fluid_point_set_index, pt_idx, nidx);
							GA_Offset fn_off = gdp.pointOffset(fn_idx);
							fluid_particles->neighbor_lists_cache[pt_off].emplace_back(fn_off);
						}
					}

					// boundary neighbors
					for (auto &pair: boundary_point_set_indices)
					{
						UT_String boundary_name = pair.first;
						unsigned int boundary_point_set_index = pair.second;
						auto neighbor_count = fluid_point_set.n_neighbors(boundary_point_set_index, pt_idx);
						for (int nidx = 0; nidx < neighbor_count; ++nidx)
						{
							auto bn_idx = fluid_point_set.neighbor(boundary_point_set_index, pt_idx, nidx);
							GA_Offset bn_off = gdp.pointOffset(bn_idx);
							fluid_particles->other_neighbor_lists_cache[boundary_name][pt_off].emplace_back(bn_off);
						}
					}
				}
		}
#endif
	} else if (getBackend() == 0) // TBBParallelHash (NOT YET SUPPORT Akinci2012)
	{
		CubbyFlow::Array1<CubbyFlow::Vector3D> positions;
		SIM_GeometryAutoWriteLock lock(fluid_particles);
		GU_Detail &gdp = lock.getGdp();
		positions.Resize(gdp.getNumPoints());
		{
			GA_Offset pt_off;
			GA_FOR_ALL_PTOFF(&gdp, pt_off)
				{
					GA_Index pt_idx = gdp.pointIndex(pt_off);
					UT_Vector3 pos = gdp.getPos3(pt_off);
					positions[pt_idx][0] = pos.x();
					positions[pt_idx][1] = pos.y();
					positions[pt_idx][2] = pos.z();
				}
		}
		CubbyFlow::Vector3UZ res = CubbyFlow::Vector3UZ::MakeConstant(64);
		auto searcher = CubbyFlow::PointParallelHashGridSearcher3::GetBuilder()
				.WithGridSpacing(spacing * 2)
				.WithResolution(res)
				.MakeShared();
		searcher->Build(positions, radius);
		{
			GA_Offset pt_off;
			GA_FOR_ALL_PTOFF(&gdp, pt_off)
				{
					GA_Index pt_idx = gdp.pointIndex(pt_off);
					UT_Vector3 pt = gdp.getPos3(pt_off);
					searcher->ForEachNearbyPoint(
							AS_CFVector3D(pt), radius, [&](size_t n_idx, const CubbyFlow::Vector3D &)
							{
								if (pt_idx != n_idx)
								{
									GA_Offset n_off = gdp.pointOffset(n_idx);
									fluid_particles->neighbor_lists_cache[pt_off].emplace_back(n_off);
								}
							});
				}
		}
	}

	return true;
}
