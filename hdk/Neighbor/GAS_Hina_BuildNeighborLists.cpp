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
	SIM_Hina_Particles *fluid_particles = SIM_DATA_CAST(getGeometryCopy(fluid_obj, GAS_NAME_GEOMETRY), SIM_Hina_Particles);
	CHECK_NULL_RETURN_VOID(fluid_particles)
	std::map<UT_String, SIM_Hina_Akinci2012BoundaryParticles *> boundary_particles = FetchAllAkinciBoundaries(fluid_obj);

	switch (getBackend())
	{
		case 0: // TBB parallel hash (NOT YET SUPPORT Akinci2012)
			break;
		case 1: // CUDA
		{
			if (!nsearch)
			{
				fluid_particles->obj_name = fluid_obj->getName();
				FetchAllAkinciBoundariesAndApply(fluid_obj, [&](SIM_Object *obj_boundary, SIM_Hina_Akinci2012BoundaryParticles *boundary_akinci, const UT_String &boundary_name)
				{
					boundary_akinci->load_sop(obj_boundary);
					boundary_akinci->obj_name = obj_boundary->getName();
				});
				init_search_engine(fluid_particles, boundary_particles);
			} else
				update_search_engine(fluid_particles, boundary_particles);
		}
			break;
		default:
			break;
	}

	return true;
}
void GAS_Hina_BuildNeighborLists::init_search_engine(SIM_Hina_Particles *particles, std::map<UT_String, SIM_Hina_Akinci2012BoundaryParticles *> &akinci_boundaries)
{
	nsearch = new cuNSearch::NeighborhoodSearch(getKernelRadius());
	cached_positions.clear();
	cached_point_set_indices.clear();

	// ========== 1. Add Fluid Particles ==========
	_add_particle_set(particles->obj_name, particles);

	// ========== 2. Add Boundaries Particles ==========
	for (auto &pair: akinci_boundaries)
		_add_particle_set(pair.first, pair.second);

	// ========== 3. Search ==========
	nsearch->set_active(true); // for first search, we search for all other point sets with all other point sets
	nsearch->find_neighbors();

	// ========== 4. update neighbor caches in Particles Data ==========
	std::map<UT_String, SIM_Hina_Particles *> _;
	_[particles->obj_name] = particles;
	for (auto &pair: akinci_boundaries)
		_[pair.first] = pair.second;
	_update_neighbor_cache(_, true);

	// ========== 5. Calculate Mass and Volume ==========
	particles->calculate_mass();
	particles->calculate_volume();
	for (auto &pair: akinci_boundaries)
	{
		pair.second->calculate_volume();
		pair.second->calculate_mass();
		pair.second->commit(); // important! for affectors, we need to manually commit to avoid solving sequence issues
	}

	/**
	 * For fluid particles, we need to search with all other particles
	 * For boundary particles, we only need to search with itself (for initialize Volume)
	 * After initialization, we only need to search neighbors for fluid particles
	 */
	for (const auto &pair1: cached_point_set_indices)
	{
		const UT_String &this_name = pair1.first;
		const unsigned int this_point_set_index = pair1.second;
		if (this_name != particles->obj_name)
			nsearch->set_active(this_point_set_index, false /* search neighbors */, true /* BE SEARCHED by other points sets*/);
	}
}
void GAS_Hina_BuildNeighborLists::update_search_engine(SIM_Hina_Particles *particles, std::map<UT_String, SIM_Hina_Akinci2012BoundaryParticles *> &akinci_boundaries)
{
	std::map<UT_String, SIM_Hina_Particles *> _;
	_[particles->obj_name] = particles;
	for (auto &pair: akinci_boundaries)
		_[pair.first] = pair.second;

	_update_particle_set(particles->obj_name, particles);
	nsearch->find_neighbors();
	_update_neighbor_cache(_);
}
void GAS_Hina_BuildNeighborLists::_add_particle_set(const UT_String &name, SIM_Hina_Particles *particles)
{
	std::vector<std::array<cuNSearch::Real, 3>> &positions = cached_positions[name];
	{
		positions.resize(particles->size());
		particles->for_each_offset(
				[&](const GA_Offset &pt_off)
				{
					GA_Index pt_idx = particles->offset2index[pt_off];
					UT_Vector3 pos = particles->position_cache[pt_off];
					positions[pt_idx][0] = pos.x();
					positions[pt_idx][1] = pos.y();
					positions[pt_idx][2] = pos.z();
				});
	}
	cached_point_set_indices[name] = nsearch->add_point_set(positions.front().data(), positions.size(), true /* dynamic */, true /* search neighbors */, true /* BE SEARCHED by other points sets*/);
}
void GAS_Hina_BuildNeighborLists::_update_particle_set(const UT_String &name, SIM_Hina_Particles *particles)
{
	std::vector<std::array<cuNSearch::Real, 3>> &positions = cached_positions[name];
	{
		positions.resize(particles->size());
		particles->for_each_offset(
				[&](const GA_Offset &pt_off)
				{
					GA_Index pt_idx = particles->offset2index[pt_off];
					UT_Vector3 pos = particles->position_cache[pt_off];
					positions[pt_idx][0] = pos.x();
					positions[pt_idx][1] = pos.y();
					positions[pt_idx][2] = pos.z();
				});
	}
	nsearch->resize_point_set(cached_point_set_indices[name], positions.front().data(), positions.size());
	nsearch->update_point_set(cached_point_set_indices[name]);
}
void GAS_Hina_BuildNeighborLists::_update_neighbor_cache(std::map<UT_String, SIM_Hina_Particles *> &target, bool init)
{
	for (auto &pair: target)
	{
		pair.second->neighbor_lists_builder = this; // TODO: remove following

		const UT_String &name = pair.first;
		SIM_Hina_Particles *particles = pair.second;

		if (!init && SIM_DATA_CAST(particles, SIM_Hina_Akinci2012BoundaryParticles)) // skip boundary particles
			continue;

		particles->neighbor_lists_cache.clear();
		particles->other_neighbor_lists_cache.clear();
		const unsigned int point_set_index = cached_point_set_indices[name];
		auto &point_set = nsearch->point_set(point_set_index);
		particles->for_each_offset(
				[&](const GA_Offset &pt_off)
				{
					GA_Index pt_idx = particles->offset2index[pt_off];
					for (const auto &pair: cached_point_set_indices)
					{
						const UT_String &other_point_set_name = pair.first;
						const unsigned int other_point_set_index = pair.second;

						auto neighbor_count = point_set.n_neighbors(other_point_set_index, pt_idx);
						for (int nidx = 0; nidx < neighbor_count; ++nidx)
						{
							const auto n_idx = point_set.neighbor(other_point_set_index, pt_idx, nidx);

							if (other_point_set_index == point_set_index) // self neighbors
							{
								const GA_Offset n_off = particles->index2offset[n_idx];
								const UT_Vector3 n_pos = UT_Vector3D{
										point_set.GetPoints()[3 * n_idx + 0],
										point_set.GetPoints()[3 * n_idx + 1],
										point_set.GetPoints()[3 * n_idx + 2]};
								ParticleState ns;
								ns.pt_off = n_off;
								ns.pt_pos = n_pos;
								particles->neighbor_lists_cache[pt_off].emplace_back(ns);
							} else // other neighbors
							{
								const GA_Offset n_off = target[other_point_set_name]->index2offset[n_idx];
								auto &other_point_set = nsearch->point_set(other_point_set_index);
								const UT_Vector3 n_pos = UT_Vector3D{
										other_point_set.GetPoints()[3 * n_idx + 0],
										other_point_set.GetPoints()[3 * n_idx + 1],
										other_point_set.GetPoints()[3 * n_idx + 2]};
								ParticleState ns;
								ns.pt_off = n_off;
								ns.pt_pos = n_pos;
								particles->other_neighbor_lists_cache[other_point_set_name][pt_off].emplace_back(ns);
							}
						}
					}
				});
	}
}
