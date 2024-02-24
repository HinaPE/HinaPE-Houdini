#include "SIM_Hina_Particles.h"
#include <Collision/SIM_Hina_RigidBodyCollider.h>

#include <CUDA_CubbyFlow/Core/Geometry/BoundingBox.hpp>
#include <CUDA_CubbyFlow/Core/Particle/SPHKernels.hpp>
#include <CUDA_CubbyFlow/Core/Particle/SPHSystemData.hpp>
#include <CUDA_CubbyFlow/Core/PointGenerator/BccLatticePointGenerator.hpp>
#include <CUDA_CubbyFlow/Core/PointGenerator/TrianglePointGenerator.hpp>
#include <CUDA_HinaPE/kernels.h>

SIM_HINA_GEOMETRY_IMPLEMENT(
		Particles,
		false,
		HINA_FLOAT_VECTOR_PARAMETER(FluidDomain, 3, 2, 2, 2) \
        HINA_FLOAT_PARAMETER(TargetSpacing, .02) \
        HINA_FLOAT_PARAMETER(KernelRadiusOverTargetSpacing, 1.8) \
        HINA_FLOAT_PARAMETER(TargetDensity, 1000.) \
        static std::array<PRM_Name, 4> Kernels = {\
            PRM_Name("0", "Poly64"), \
            PRM_Name("1", "Spiky"), \
            PRM_Name("2", "CubicSpline"), \
            PRM_Name(nullptr), \
}; \
        static PRM_Name KernelName("Kernel", "Kernel"); \
        static PRM_Default KernelNameDefault(2, "CubicSpline"); \
        static PRM_ChoiceList CL(PRM_CHOICELIST_SINGLE, Kernels.data()); \
        PRMS.emplace_back(PRM_ORD, 1, &KernelName, &KernelNameDefault, &CL); \
)
void SIM_Hina_Particles::_init_Particles()
{
	this->Mass = 1;
	this->neighbor_lists_cache.clear();
	this->other_neighbor_lists_cache.clear();
	this->positions_cache.clear();
	this->velocity_cache.clear();
}
void SIM_Hina_Particles::_makeEqual_Particles(const SIM_Hina_Particles *src)
{
	this->Mass = src->Mass;
	this->neighbor_lists_cache = src->neighbor_lists_cache;
	this->other_neighbor_lists_cache = src->other_neighbor_lists_cache;
	this->positions_cache = src->positions_cache;
	this->velocity_cache = src->velocity_cache;
}
void SIM_Hina_Particles::_setup_gdp(GU_Detail *gdp) const
{
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_VELOCITY, HINA_GEOMETRY_ATTRIBUTE_TYPE_VECTOR3)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_FORCE, HINA_GEOMETRY_ATTRIBUTE_TYPE_VECTOR3)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_MASS, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_DENSITY, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_PRESSURE, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_VOLUME, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_NEIGHBOR_SUM_SELF, HINA_GEOMETRY_ATTRIBUTE_TYPE_INT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_NEIGHBOR_SUM_OTHERS, HINA_GEOMETRY_ATTRIBUTE_TYPE_INT)
}
void SIM_Hina_Particles::commit()
{
	SIM_GeometryAutoWriteLock lock(this);
	GU_Detail &gdp = lock.getGdp();
	GA_RWHandleV3 pos_handle = gdp.getP();
	GA_RWHandleV3 vel_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VELOCITY);
	GA_Offset pt_off;
	GA_FOR_ALL_PTOFF(&gdp, pt_off)
		{
			UT_Vector3 pos = positions_cache[pt_off];
			UT_Vector3 vel = velocity_cache[pt_off];
			pos_handle.set(pt_off, pos);
			vel_handle.set(pt_off, vel);
		}
}
void SIM_Hina_Particles::calculate_mass()
{
	fpreal TargetSpacing = getTargetSpacing();
	fpreal TargetDensity = getTargetDensity();
	fpreal KernelRadius = getKernelRadiusOverTargetSpacing() * getTargetSpacing();
	// Compute Initial Mass
	{
		using namespace CubbyFlow;

		Array1<Vector3D> points;
		BccLatticePointGenerator pointsGenerator;
		BoundingBox3D sampleBound{
				Vector3D::MakeConstant(-1.5 * KernelRadius),
				Vector3D::MakeConstant(1.5 * KernelRadius)
		};

		pointsGenerator.Generate(sampleBound, TargetSpacing, &points);

		double maxNumberDensity = 0.0;
//		SPHStdKernel3 kernel{KernelRadius};
		HinaPE::CubicSplineKernel<false> kernel(KernelRadius);

		for (size_t i = 0; i < points.Length(); ++i)
		{
			const Vector3D &point = points[i];
			double sum = 0.0;

			for (size_t j = 0; j < points.Length(); ++j)
			{
				const Vector3D &neighborPoint = points[j];
//				sum += kernel(neighborPoint.DistanceTo(point));
				sum += kernel.kernel(neighborPoint.DistanceTo(point));
			}

			maxNumberDensity = std::max(maxNumberDensity, sum);
		}

		this->Mass = TargetDensity / maxNumberDensity;
	}

	SIM_GeometryAutoWriteLock lock(this);
	GU_Detail &gdp = lock.getGdp();
	GA_RWHandleF mass_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_MASS);
	GA_Offset pt_off;
	GA_FOR_ALL_PTOFF(&gdp, pt_off)
		{
			mass_handle.set(pt_off, this->Mass);
		}
}
void SIM_Hina_Particles::calculate_volume()
{
	HinaPE::CubicSplineKernel<false> kernel(getTargetSpacing() * getKernelRadiusOverTargetSpacing());
	SIM_GeometryAutoWriteLock lock(this);
	GU_Detail &gdp = lock.getGdp();
	GA_Offset pt_off;
	GA_RWHandleF volume_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VOLUME);
	GA_FOR_ALL_PTOFF(&gdp, pt_off)
		{
			fpreal volume = 0.0;
			volume += kernel.kernel(0); // remember to include self (self is also a neighbor of itself)
			for_each_neighbor_self(pt_off, [&](const GA_Offset &n_off, const UT_Vector3 &)
			{
				UT_Vector3 p_i = gdp.getPos3(pt_off);
				UT_Vector3 p_j = gdp.getPos3(n_off);
				const UT_Vector3 r = p_i - p_j;
				volume += kernel.kernel(r.length());
			});
			volume = 1.0 / volume;
			volume_handle.set(pt_off, volume);
		}
}
void SIM_Hina_Particles::for_each_neighbor_self(const GA_Offset &pt_off, std::function<void(const GA_Offset &, const UT_Vector3 &)> func)
{
	const auto &neighbors = neighbor_lists_cache[pt_off];
	for (const auto &neighbor: neighbors)
		func(neighbor.pt_off, neighbor.pt_pos);
}
void SIM_Hina_Particles::for_each_neighbor_others(const GA_Offset &pt_off, std::function<void(const GA_Offset &, const UT_Vector3 &)> func)
{
	for (auto &neighbor_lists: other_neighbor_lists_cache)
		for (const auto &neighbor: neighbor_lists.second[pt_off])
			func(neighbor.pt_off, neighbor.pt_pos);
}
void SIM_Hina_Particles::for_each_neighbor_others(const GA_Offset &pt_off, std::function<void(const GA_Offset &, const UT_Vector3 &)> func, const UT_String &other_name)
{
	const auto &neighbors = other_neighbor_lists_cache[other_name][pt_off];
	for (const auto &neighbor: neighbors)
		func(neighbor.pt_off, neighbor.pt_pos);
}

GAS_HINA_SUBSOLVER_IMPLEMENT(
		CommitCache,
		true,
		true,
		TARGET_PARTICLE_GEOMETRY(SIM_Hina_Particles)
)
void GAS_Hina_CommitCache::_init() {}
void GAS_Hina_CommitCache::_makeEqual(const GAS_Hina_CommitCache *src) {}
bool GAS_Hina_CommitCache::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_Hina_Particles *particles = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_Particles);
	CHECK_NULL_RETURN_BOOL(particles)
	particles->commit();
	return true;
}
