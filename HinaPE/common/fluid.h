#ifndef HINAPE_COMMON1_H
#define HINAPE_COMMON1_H

#include <functional>
#include <memory>

#include <CUDA_CubbyFlow/Core/Emitter/VolumeParticleEmitter3.hpp>
#include <CUDA_CubbyFlow/Core/Geometry/SurfaceToImplicit.hpp>
#include <CUDA_CubbyFlow/Core/PointGenerator/BccLatticePointGenerator.hpp>
#include <CUDA_CubbyFlow/Core/Searcher/PointHashGridSearcher.hpp>
#include <CUDA_CubbyFlow/Core/Searcher/PointParallelHashGridSearcher.hpp>
#include <CUDA_CubbyFlow/Core/Utils/Samplers.hpp>
#include <CUDA_CubbyFlow/Core/Geometry/TriangleMesh3.hpp>
#include <CUDA_CubbyFlow/Core/Geometry/ImplicitSurfaceSet.hpp>
#include <CUDA_CubbyFlow/Core/Utils/Logging.hpp>

namespace HinaPE
{
inline void serial_for(size_t n, const std::function<void(size_t)> &f) { for (size_t i = 0; i < n; ++i) { f(i); }}
#if true
inline void parallel_for(size_t n, const std::function<void(size_t)> &f) { serial_for(n, f); }
#else
inline void parallel_for(size_t n, const std::function<void(size_t)> &f){ tbb::parallel_for(size_t(0), n, [&](size_t i) { f(i); }); }
#endif

template<typename real, typename Vector3, typename ScalarArray, typename Vector3Array>
struct IFluid
{
	size_t size;
	Vector3Array x;
	Vector3Array v;
	Vector3Array a;
	ScalarArray m;
	ScalarArray V;
	ScalarArray rho;
	ScalarArray neighbor_this;
	ScalarArray neighbor_others;
};

template<typename real, typename Vector3, typename ScalarArray, typename Vector3Array>
struct IAkinciBoundary
{
	size_t size;
	Vector3Array x;
	Vector3Array v;
	Vector3Array f;
	ScalarArray m;
	ScalarArray V;
	ScalarArray rho;

	ScalarArray neighbor_this;
};

template<typename real, typename Vector3, typename ScalarArray, typename Vector3Array>
struct IFluidEmitter
{
	static void UseCubbyVolumeEmitter(Vector3Array* TARGET, CubbyFlow::ImplicitSurfaceSet3Ptr Implicit, real TargetSpacing, real MaxParticles)
	{
		CubbyFlow::Logging::Mute();
		const double maxJitterDist = 0;

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
				TARGET->push_back({candidate.x, candidate.y, candidate.z});
			}
			return true;
		});
	}

	static void UseFluidBlock(Vector3Array* TARGET, const Vector3 &Start, const Vector3 &End, const real TargetSpacing)
	{
		TARGET->clear();
		for (real x = Start.x(); x <= End.x(); x += TargetSpacing)
			for (real y = Start.y(); y <= End.y(); y += TargetSpacing)
				for (real z = Start.z(); z <= End.z(); z += TargetSpacing)
					TARGET->emplace_back(x, y, z);
	}
};

} // namespace HinaPE

#endif //HINAPE_COMMON1_H
