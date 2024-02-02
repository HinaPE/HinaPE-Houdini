#include "GAS_Hina_VolumeParticleEmitter.h"
#include <Common/SIM_Hina_Particles.h>

#include <CUDA_CubbyFlow/Core/Emitter/VolumeParticleEmitter3.hpp>
#include <CUDA_CubbyFlow/Core/Geometry/SurfaceToImplicit.hpp>
#include <CUDA_CubbyFlow/Core/PointGenerator/BccLatticePointGenerator.hpp>
#include <CUDA_CubbyFlow/Core/Searcher/PointHashGridSearcher.hpp>
#include <CUDA_CubbyFlow/Core/Searcher/PointParallelHashGridSearcher.hpp>
#include <CUDA_CubbyFlow/Core/Utils/Samplers.hpp>
#include <CUDA_CubbyFlow/Core/Utils/Logging.hpp>
#include <CUDA_CubbyFlow/Core/Geometry/TriangleMesh3.hpp>
#include <CUDA_CubbyFlow/Core/Geometry/ImplicitSurfaceSet.hpp>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		VolumeParticleEmitter,
		true,
		false,
		TARGET_PARTICLE_GEOMETRY(SIM_Hina_Particles) \
        HINA_FLOAT_VECTOR_PARAMETER(FluidDomain, 3, 2, 2, 2) \
        HINA_FLOAT_PARAMETER(TargetSpacing, .02) \
        HINA_FLOAT_PARAMETER(Jitter, .0) \
        HINA_INT_PARAMETER(MaxNumOfParticles, 100000) \
        HINA_BOOL_PARAMETER(IsOneShot, true) \
        HINA_BOOL_PARAMETER(IsEmitterMove, false) \
)

CubbyFlow::Array1<CubbyFlow::Surface3Ptr> _search_source_geometry(SIM_Data *data)
{
	CubbyFlow::Array1<CubbyFlow::Surface3Ptr> res;



	// For performance, We only support upto 1 external SIM_Geometry
	SIM_Geometry *src_geo = SIM_DATA_GET(*data, SIM_GEOMETRY_DATANAME, SIM_Geometry);
	if (src_geo)
	{
		SIM_GeometryAutoReadLock lock(src_geo);
		const GU_Detail *gdp_source = lock.getGdp();

		CubbyFlow::TriangleMesh3::PointArray points;
		CubbyFlow::TriangleMesh3::IndexArray point_indices;
		{
			GA_Offset pt_off;
			GA_FOR_ALL_PTOFF(gdp_source, pt_off)
				{
					const UT_Vector3 pos = gdp_source->getPos3(pt_off);
					points.Append({pos.x(), pos.y(), pos.z()});
				}

			const GEO_Primitive *prim;
			GA_FOR_ALL_PRIMITIVES(gdp_source, prim)
			{
				const auto *poly = dynamic_cast<const GEO_PrimPoly *>(prim);

				// Triangulate Polygon
				std::vector<size_t> polyIndices;
				for (int vi = 0; vi < poly->getVertexCount(); ++vi)
					polyIndices.push_back(poly->getPointIndex(vi));
				for (size_t i = 1; i < polyIndices.size() - 1; ++i)
					point_indices.Append({polyIndices[0], polyIndices[i + 1], polyIndices[i]}); // notice the normal
			}
		}
		CubbyFlow::TriangleMesh3Ptr mesh = CubbyFlow::TriangleMesh3::GetBuilder().WithPoints(points).WithPointIndices(point_indices).MakeShared();

		res.Append(mesh);
	}

	return res;
}
void GAS_Hina_VolumeParticleEmitter::_init()
{
	this->Emitted = false;
	this->EmittedParticles = 0;
}
void GAS_Hina_VolumeParticleEmitter::_makeEqual(const GAS_Hina_VolumeParticleEmitter *src)
{
	this->Emitted = src->Emitted;
	this->EmittedParticles = src->EmittedParticles;
}
bool GAS_Hina_VolumeParticleEmitter::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	bool IsOneShot = getIsOneShot();
	bool IsEmitterMove = getIsEmitterMove();
	size_t MaxNumberOfParticles = getMaxNumOfParticles();
	UT_Vector3 FluidDomain = getFluidDomain();
	fpreal TargetSpacing = getTargetSpacing();
	fpreal Jitter = getJitter();

	if (IsOneShot && Emitted)
		return true;

	SIM_Hina_Particles *particles = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_Particles);
	CHECK_NULL_RETURN_BOOL(particles)

	// Fetch Existing Particles
	UT_Vector3Array exist_positions;
	{
		SIM_GeometryAutoWriteLock lock(particles);
		GU_Detail &gdp = lock.getGdp();
		GA_Offset pt_pff;
		GA_FOR_ALL_PTOFF(&gdp, pt_pff)
			{
				exist_positions.append(gdp.getPos3(pt_pff));
			}
	}

	// Generate New Particles
	UT_Vector3Array new_positions;
	UT_Vector3Array new_velocities;
	GA_Size new_particles{};
	{
		using namespace CubbyFlow;
		Logging::Mute();

		// Build Surface
		CubbyFlow::Array1<CubbyFlow::Surface3Ptr> surfaces = _search_source_geometry(this);
		if (surfaces.IsEmpty())
		{
			error_msg.appendSprintf("NO Source Geometry, From %s\n", DATANAME);
			return false;
		}
		CubbyFlow::ImplicitSurfaceSet3Ptr implicit = CubbyFlow::ImplicitSurfaceSet3::GetBuilder().WithExplicitSurfaces(surfaces).MakeShared();

		const double maxJitterDist = 0.5 * Jitter * TargetSpacing;
		UT_Vector3 HalfFluidDomain = FluidDomain;
		HalfFluidDomain /= 2;
		BoundingBox3D region;
		region.lowerCorner = AS_CFVector3D(-HalfFluidDomain);
		region.upperCorner = AS_CFVector3D(HalfFluidDomain);
		if (implicit->IsBounded())
		{
			const BoundingBox3D surfaceBBox = implicit->GetBoundingBox();
			region.lowerCorner = Max(region.lowerCorner, surfaceBBox.lowerCorner);
			region.upperCorner = Min(region.upperCorner, surfaceBBox.upperCorner);
		}

		implicit->UpdateQueryEngine();

		int DEFAULT_HASH_GRID_RESOLUTION = 64;
		PointParallelHashGridSearcher3 neighbor_searcher(
				Vector3UZ(DEFAULT_HASH_GRID_RESOLUTION,
						  DEFAULT_HASH_GRID_RESOLUTION,
						  DEFAULT_HASH_GRID_RESOLUTION),
				2.0 * TargetSpacing);

		Array1<Vector3D> pos_array;
		for (int i = 0; i < exist_positions.size(); ++i)
			pos_array.Append(AS_CFVector3D(exist_positions[i]));
		neighbor_searcher.Build(pos_array);

		// Point Generator
		auto points_gen = std::make_shared<BccLatticePointGenerator>();
		std::uniform_real_distribution<> d{0.0, 1.0};
		std::mt19937 m_rng;
		points_gen->ForEachPoint(region, TargetSpacing, [&](const Vector3D &point)
		{
			const Vector3D randomDir =
					UniformSampleSphere(d(m_rng), d(m_rng));
			const Vector3D offset = maxJitterDist * randomDir;
			const Vector3D candidate = point + offset;

			if (implicit->IsInside(candidate) && !neighbor_searcher.HasNearbyPoint(candidate, TargetSpacing))
			{
				if (this->EmittedParticles >= MaxNumberOfParticles)
					return false;
				new_positions.append(AS_UTVector3D(candidate));
				new_velocities.append(AS_UTVector3D(Vector3D()));
				this->Emitted = true;
				++this->EmittedParticles;
			}
			return true;
		});
		new_particles = new_positions.size();
	}

	// Write GDP
	SIM_GeometryAutoWriteLock lock(particles);
	GU_Detail &gdp = lock.getGdp();
	GA_RWHandleV3 pos_handle = gdp.getP();
	GA_RWHandleV3 vel_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VELOCITY);
	GA_RWHandleF mass_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_MASS);
	fpreal mass = particles->Mass;
	for (GA_Size idx = 0; idx < new_particles; ++idx)
	{
		GA_Offset pt_off = gdp.appendPoint();
		UT_Vector3 pos = new_positions[idx];
		UT_Vector3 vel = new_velocities[idx];
		pos_handle.set(pt_off, pos);
		vel_handle.set(pt_off, vel);
		mass_handle.set(pt_off, mass);
	}
	return true;
}
