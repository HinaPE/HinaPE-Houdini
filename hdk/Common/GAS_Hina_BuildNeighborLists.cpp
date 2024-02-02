#include "GAS_Hina_BuildNeighborLists.h"
#include <Common/SIM_Hina_Particles.h>

#ifdef WIN32
#include "cuNSearch.h"
#else
#include <Common/CUDA_CubbyFlow/Core/Searcher/PointParallelHashGridSearcher.hpp>
#endif

GAS_HINA_SUBSOLVER_IMPLEMENT(
		BuildNeighborLists,
		true,
		false,
		TARGET_PARTICLE_GEOMETRY(SIM_Hina_Particles)
				HINA_FLOAT_PARAMETER(KernelRadius, 0.036)
)

void GAS_Hina_BuildNeighborLists::_init() {}
void GAS_Hina_BuildNeighborLists::_makeEqual(const GAS_Hina_BuildNeighborLists *src) {}
bool GAS_Hina_BuildNeighborLists::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_Hina_Particles *particles = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_Particles);
	CHECK_NULL_RETURN_BOOL(particles)
	double spacing = particles->getTargetSpacing();
	double radius = getKernelRadius();
	particles->neighbor_lists_cache.clear();

#ifdef WIN32
	std::vector<std::array<cuNSearch::Real, 3>> positions;
	SIM_GeometryAutoWriteLock lock(particles);
	GU_Detail &gdp = lock.getGdp();
	positions.resize(gdp.getNumPoints());
	GA_Offset pt_off;
	{
		GA_FOR_ALL_PTOFF(&gdp, pt_off)
			{
				GA_Index pt_idx = gdp.pointIndex(pt_off);
				UT_Vector3 pos = gdp.getPos3(pt_off);
				positions[pt_idx][0] = pos.x();
				positions[pt_idx][1] = pos.y();
				positions[pt_idx][2] = pos.z();
			}
	}

	cuNSearch::NeighborhoodSearch nsearch(radius);
	auto pointSetIndex = nsearch.add_point_set(positions.front().data(), positions.size(), true, true);
	nsearch.find_neighbors();
	auto &pointSet = nsearch.point_set(pointSetIndex);

	{
		GA_FOR_ALL_PTOFF(&gdp, pt_off)
			{
				GA_Index pt_idx = gdp.pointIndex(pt_off);
				auto neighbor_count = pointSet.n_neighbors(pointSetIndex, pt_idx);
				for (int nidx = 0; nidx < neighbor_count; ++nidx)
				{
					auto t_idx = pointSet.neighbor(pointSetIndex, pt_idx, nidx);
					GA_Offset n_off = gdp.pointOffset(t_idx);
					particles->neighbor_lists_cache[pt_off].emplace_back(n_off);
				}
			}
	}
#else
	CubbyFlow::Array1<CubbyFlow::Vector3D> positions;
	SIM_GeometryAutoWriteLock lock(particles);
	GU_Detail &gdp = lock.getGdp();
	positions.Resize(gdp.getNumPoints());
	GA_Offset pt_off;
	{
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
								particles->neighbor_lists_cache[pt_off].emplace_back(n_off);
							}
						});
			}
	}
#endif

	return true;
}
