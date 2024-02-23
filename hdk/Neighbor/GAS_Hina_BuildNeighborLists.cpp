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

void GAS_Hina_BuildNeighborLists::_init()
{
	if (this->nsearch)
	{
		delete this->nsearch;
		this->nsearch = nullptr;
	}
	this->nsearch = nullptr;
	this->cached_positions.clear();
	this->cached_point_set_indices.clear();
}
void GAS_Hina_BuildNeighborLists::_makeEqual(const GAS_Hina_BuildNeighborLists *src)
{
	this->nsearch = src->nsearch;
	this->cached_positions = src->cached_positions;
	this->cached_point_set_indices = src->cached_point_set_indices;
}
bool GAS_Hina_BuildNeighborLists::_solve(SIM_Engine &engine, SIM_Object *fluid_obj, SIM_Time time, SIM_Time timestep)
{
	if (getBackend() == 1) // CUDA
	{
#ifdef WIN32
		if (!nsearch)
			init_search_engine(fluid_obj);
		else
		{
			SIM_Hina_Particles *fluid_particles = SIM_DATA_CAST(getGeometryCopy(fluid_obj, GAS_NAME_GEOMETRY), SIM_Hina_Particles);
			CHECK_NULL_RETURN_BOOL(fluid_particles)
			update_particle_set(fluid_obj->getName(), fluid_particles);
			nsearch->find_neighbors();
			update_neighbor(fluid_obj->getName(), fluid_particles);
		}
#endif
	} else if (getBackend() == 0) // TBBParallelHash (NOT YET SUPPORT Akinci2012)
	{
//		CubbyFlow::Array1<CubbyFlow::Vector3D> positions;
//		SIM_GeometryAutoWriteLock lock(fluid_particles);
//		GU_Detail &gdp = lock.getGdp();
//		positions.Resize(gdp.getNumPoints());
//		{
//			GA_Offset pt_off;
//			GA_FOR_ALL_PTOFF(&gdp, pt_off)
//				{
//					GA_Index pt_idx = gdp.pointIndex(pt_off);
//					UT_Vector3 pos = gdp.getPos3(pt_off);
//					positions[pt_idx][0] = pos.x();
//					positions[pt_idx][1] = pos.y();
//					positions[pt_idx][2] = pos.z();
//				}
//		}
//		CubbyFlow::Vector3UZ res = CubbyFlow::Vector3UZ::MakeConstant(64);
//		auto searcher = CubbyFlow::PointParallelHashGridSearcher3::GetBuilder()
//				.WithGridSpacing(spacing * 2)
//				.WithResolution(res)
//				.MakeShared();
//		searcher->Build(positions, radius);
//		{
//			GA_Offset pt_off;
//			GA_FOR_ALL_PTOFF(&gdp, pt_off)
//				{
//					GA_Index pt_idx = gdp.pointIndex(pt_off);
//					UT_Vector3 pt = gdp.getPos3(pt_off);
//					searcher->ForEachNearbyPoint(
//							AS_CFVector3D(pt), radius, [&](size_t n_idx, const CubbyFlow::Vector3D &)
//							{
//								if (pt_idx != n_idx)
//								{
//									GA_Offset n_off = gdp.pointOffset(n_idx);
//									fluid_particles->neighbor_lists_cache[pt_off].emplace_back(n_off);
//								}
//							});
//				}
//		}
	}

	return true;
}
void GAS_Hina_BuildNeighborLists::init_search_engine(SIM_Object *fluid_obj)
{
	nsearch = new cuNSearch::NeighborhoodSearch(getKernelRadius());
	cached_positions.clear();
	cached_point_set_indices.clear();

	SIM_Hina_Particles *fluid_particles = SIM_DATA_CAST(getGeometryCopy(fluid_obj, GAS_NAME_GEOMETRY), SIM_Hina_Particles);
	CHECK_NULL_RETURN_VOID(fluid_particles)

	// ========== 1. Add Fluid Particles ==========
	_add_particle_set(fluid_obj->getName(), fluid_particles);

	// ========== 2. Add Boundaries Particles ==========
	SIM_ObjectArray affectors;
	fluid_obj->getAffectors(affectors, "SIM_RelationshipCollide");
	exint num_affectors = affectors.entries();
	std::map<UT_String, SIM_Hina_Akinci2012BoundaryParticles *> boundary_particles;
	for (int i = 0; i < num_affectors; ++i)
	{
		SIM_Object *obj_collider = affectors(i);
		if (obj_collider->getName().equal(fluid_obj->getName()))
			continue;

		UT_String boundary_obj_name = obj_collider->getName();
		SIM_Hina_Akinci2012BoundaryParticles *boundary_akinci = SIM_DATA_GET(*obj_collider, SIM_Hina_Akinci2012BoundaryParticles::DATANAME, SIM_Hina_Akinci2012BoundaryParticles);
		if (boundary_akinci)
		{
			boundary_particles[boundary_obj_name] = boundary_akinci;
			boundary_akinci->UpdateBoundaryParticlesFromSOP(obj_collider);
			_add_particle_set(boundary_obj_name, boundary_akinci);
		}
	}
	nsearch->set_active(true); // for first search, we search for all other point sets with all other point sets
	nsearch->find_neighbors();
	update_neighbor(fluid_obj->getName(), fluid_particles);
	for (const auto &pair: boundary_particles)
		update_neighbor(pair.first, pair.second);

	/**
	 * For fluid particles, we need to search with all other particles
	 * For boundary particles, we only need to search with itself (for initialize Volume)
	 * After initialization, we only need to search neighbors for fluid particles
	 */
	for (const auto &pair1: cached_point_set_indices)
	{
		const UT_String &this_name = pair1.first;
		const unsigned int this_point_set_index = pair1.second;
		if (this_name != fluid_obj->getName())
			nsearch->set_active(this_point_set_index, false /* search neighbors */, true /* BE SEARCHED by other points sets*/);
	}
}
void GAS_Hina_BuildNeighborLists::_add_particle_set(const UT_String &name, SIM_Hina_Particles *particles)
{
	std::vector<std::array<cuNSearch::Real, 3>> &positions = cached_positions[name];
	{
		SIM_GeometryAutoReadLock lock(particles);
		const GU_Detail *gdp = lock.getGdp();
		positions.resize(gdp->getNumPoints());
		GA_Offset pt_off;
		GA_FOR_ALL_PTOFF(gdp, pt_off)
			{
				GA_Index pt_idx = gdp->pointIndex(pt_off);
				UT_Vector3 pos = gdp->getPos3(pt_off);
				positions[pt_idx][0] = pos.x();
				positions[pt_idx][1] = pos.y();
				positions[pt_idx][2] = pos.z();
			}
	}
	cached_point_set_indices[name] = nsearch->add_point_set(positions.front().data(), positions.size(), true /* dynamic */, true /* search neighbors */, true /* BE SEARCHED by other points sets*/);
}
void GAS_Hina_BuildNeighborLists::update_particle_set(const UT_String &name, SIM_Hina_Particles *particles)
{
	std::vector<std::array<cuNSearch::Real, 3>> &positions = cached_positions[name];
	{
		SIM_GeometryAutoReadLock lock(particles);
		const GU_Detail *gdp = lock.getGdp();
		positions.resize(gdp->getNumPoints());
		GA_Offset pt_off;
		GA_FOR_ALL_PTOFF(gdp, pt_off)
			{
				GA_Index pt_idx = gdp->pointIndex(pt_off);
				UT_Vector3 pos = gdp->getPos3(pt_off);
				positions[pt_idx][0] = pos.x();
				positions[pt_idx][1] = pos.y();
				positions[pt_idx][2] = pos.z();
			}
	}
	nsearch->resize_point_set(cached_point_set_indices[name], positions.front().data(), positions.size());
	nsearch->update_point_set(cached_point_set_indices[name]);
}
void GAS_Hina_BuildNeighborLists::update_neighbor(const UT_String &name, SIM_Hina_Particles *particles)
{
	particles->neighbor_lists_cache.clear();
	particles->other_neighbor_lists_cache.clear();

	const unsigned int point_set_index = cached_point_set_indices[name];
	auto &point_set = nsearch->point_set(point_set_index);
	SIM_GeometryAutoWriteLock lock(particles);
	GU_Detail &gdp = lock.getGdp();
	GA_RWHandleI self_n_sum_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_NEIGHBOR_SUM_SELF);
	GA_RWHandleI other_n_sum_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_NEIGHBOR_SUM_OTHERS);
	GA_Offset pt_off;
	GA_FOR_ALL_PTOFF(&gdp, pt_off)
		{
			GA_Index pt_idx = gdp.pointIndex(pt_off);

			for (const auto &pair: cached_point_set_indices)
			{
				const UT_String &other_point_set_name = pair.first;
				const unsigned int other_point_set_index = pair.second;

				auto neighbor_count = point_set.n_neighbors(other_point_set_index, pt_idx);
				for (int nidx = 0; nidx < neighbor_count; ++nidx)
				{
					const auto n_idx = point_set.neighbor(other_point_set_index, pt_idx, nidx);
					const GA_Offset n_off = gdp.pointOffset(n_idx);

					if (other_point_set_index == point_set_index) // self neighbors
					{
						const UT_Vector3 n_pos = UT_Vector3D{
								point_set.GetPoints()[3 * n_idx + 0],
								point_set.GetPoints()[3 * n_idx + 1],
								point_set.GetPoints()[3 * n_idx + 2]};
						particles->neighbor_lists_cache[pt_off].emplace_back(std::make_pair(n_off, n_pos));
					} else // other neighbors
					{
						auto &other_point_set = nsearch->point_set(other_point_set_index);
						const UT_Vector3 n_pos = UT_Vector3D{
								other_point_set.GetPoints()[3 * n_idx + 0],
								other_point_set.GetPoints()[3 * n_idx + 1],
								other_point_set.GetPoints()[3 * n_idx + 2]};
						particles->other_neighbor_lists_cache[other_point_set_name][pt_off].emplace_back(std::make_pair(n_off, n_pos));
					}
				}
			}

			int fn_sum = particles->neighbor_lists_cache[pt_off].size();
			int bn_sum = 0;
			for (auto &pair: particles->other_neighbor_lists_cache)
				bn_sum += pair.second[pt_off].size();
			self_n_sum_handle.set(pt_off, fn_sum);
			other_n_sum_handle.set(pt_off, bn_sum);
		}

	particles->calculate_mass();
	particles->calculate_volume();
}
