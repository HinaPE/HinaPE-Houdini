#ifndef HINAPE_EMITTER_H
#define HINAPE_EMITTER_H

#include <CUDA_CubbyFlow/Core/Emitter/VolumeParticleEmitter3.hpp>
#include <CUDA_CubbyFlow/Core/Geometry/SurfaceToImplicit.hpp>
#include <CUDA_CubbyFlow/Core/PointGenerator/BccLatticePointGenerator.hpp>
#include <CUDA_CubbyFlow/Core/Searcher/PointHashGridSearcher.hpp>
#include <CUDA_CubbyFlow/Core/Searcher/PointParallelHashGridSearcher.hpp>
#include <CUDA_CubbyFlow/Core/Utils/Samplers.hpp>
#include <CUDA_CubbyFlow/Core/Geometry/TriangleMesh3.hpp>
#include <CUDA_CubbyFlow/Core/Geometry/ImplicitSurfaceSet.hpp>
#include <CUDA_CubbyFlow/Core/Utils/Logging.hpp>

#include "geometry.h"

#include <UT/UT_Quaternion.h>
using Quaternion = UT_Quaternion;

template<typename real, typename Vector3, typename ScalarArray, typename Vector3Array>
struct IFluidEmitter
{
	static void UseTriangleMeshSource(Vector3Array *TARGET, const std::pair<std::vector<Vector3>, std::vector<size_t>> &triangle_mesh_info, Vector3 pos, real TargetSpacing, real MaxParticles)
	{
		CubbyFlow::Logging::Mute();
		const double maxJitterDist = 0;

		const std::vector<Vector3> &vertices = triangle_mesh_info.first;
		const std::vector<size_t> &indices = triangle_mesh_info.second;
		CubbyFlow::TriangleMesh3::PointArray points;
		CubbyFlow::TriangleMesh3::IndexArray point_indices;
		for (size_t i = 0; i < vertices.size(); i++)
			points.Append(CubbyFlow::Vector3D{vertices[i].x(), vertices[i].y(), vertices[i].z()});
		for (size_t i = 0; i < indices.size(); i += 3)
			point_indices.Append(CubbyFlow::Vector3UZ{indices[i], indices[i + 1], indices[i + 2]});
		auto surface = CubbyFlow::TriangleMesh3::GetBuilder().WithPoints(points).WithPointIndices(point_indices).MakeShared();
		surface->transform = CubbyFlow::Transform3{CubbyFlow::Vector3D{pos.x(), pos.y(), pos.z()}, CubbyFlow::QuaternionD{1, 0, 0, 0}};
		CubbyFlow::Array1<CubbyFlow::Surface3Ptr> surfaces;
		surfaces.Append(surface);
		CubbyFlow::ImplicitSurfaceSet3Ptr Implicit = CubbyFlow::ImplicitSurfaceSet3::GetBuilder().WithExplicitSurfaces(surfaces).MakeShared();
		Implicit->UpdateQueryEngine();

		constexpr int DEFAULT_HASH_GRID_RESOLUTION = 64;
		CubbyFlow::PointParallelHashGridSearcher3 neighbor_searcher(
				CubbyFlow::Vector3UZ(DEFAULT_HASH_GRID_RESOLUTION,
									 DEFAULT_HASH_GRID_RESOLUTION,
									 DEFAULT_HASH_GRID_RESOLUTION),
				2. * TargetSpacing);

		CubbyFlow::Array1<CubbyFlow::Vector3D> pos_array;
		for (int i = 0; i < TARGET->size(); ++i)
			pos_array.Append({(*TARGET)[i].z(), (*TARGET)[i].z(), (*TARGET)[i].z()});
		neighbor_searcher.Build(pos_array);

		auto points_gen = std::make_shared<CubbyFlow::BccLatticePointGenerator>();
		std::mt19937 m_rng(0);

		CubbyFlow::BoundingBox3D region = Implicit->GetBoundingBox();
		points_gen->ForEachPoint(region, TargetSpacing, [&](const CubbyFlow::Vector3D &point)
		{
			std::uniform_real_distribution<> d1{0.0, 1.0};
			std::uniform_real_distribution<> d2{0.0, 1.0};
			const CubbyFlow::Vector3D randomDir =
					CubbyFlow::UniformSampleSphere(d1(m_rng), d2(m_rng));
			const CubbyFlow::Vector3D offset = maxJitterDist * randomDir;
			const CubbyFlow::Vector3D candidate = point + offset;

			if (Implicit->IsInside(candidate) && !neighbor_searcher.HasNearbyPoint(candidate, TargetSpacing))
			{
				if (TARGET->size() >= MaxParticles)
					return false;
				TARGET->push_back({static_cast<real>(candidate.x), static_cast<real>(candidate.y), static_cast<real>(candidate.z)});
			}
			return true;
		});
	}

	static void UseFluidBlock(Vector3Array *TARGET, const Vector3 &Start, const Vector3 &End, const real TargetSpacing)
	{
		TARGET->clear();
		for (real x = Start.x(); x <= End.x(); x += TargetSpacing)
			for (real y = Start.y(); y <= End.y(); y += TargetSpacing)
				for (real z = Start.z(); z <= End.z(); z += TargetSpacing)
					TARGET->emplace_back(x, y, z);
	}
};

#endif //HINAPE_EMITTER_H
