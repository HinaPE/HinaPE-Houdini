#include "SIM_Hina_Particles.h"

#include <Neighbor/GAS_Hina_BuildNeighborLists.h>
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
		HINA_FLOAT_VECTOR_PARAMETER(Gravity, 3, 0, -9.8, 0)
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
	this->UnivMass = 1;
	this->neighbor_lists_cache.clear();
	this->other_neighbor_lists_cache.clear();
	this->offset2index.clear();
	this->index2offset.clear();
	this->position_cache.clear();
	this->velocity_cache.clear();
	this->force_cache.clear();
	this->mass_cache.clear();
	this->volume_cache.clear();
	this->density_cache.clear();
	this->obj_name = "";
	this->neighbor_lists_builder = nullptr;
}
void SIM_Hina_Particles::_makeEqual_Particles(const SIM_Hina_Particles *src)
{
	this->UnivMass = src->UnivMass;
	this->neighbor_lists_cache = src->neighbor_lists_cache;
	this->other_neighbor_lists_cache = src->other_neighbor_lists_cache;
	this->offset2index = src->offset2index;
	this->index2offset = src->index2offset;
	this->position_cache = src->position_cache;
	this->velocity_cache = src->velocity_cache;
	this->force_cache = src->force_cache;
	this->mass_cache = src->mass_cache;
	this->volume_cache = src->volume_cache;
	this->density_cache = src->density_cache;
	this->obj_name = src->obj_name;
	this->neighbor_lists_builder = src->neighbor_lists_builder;
}
void SIM_Hina_Particles::_setup_gdp(GU_Detail *gdp) const
{
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_VELOCITY, HINA_GEOMETRY_ATTRIBUTE_TYPE_VECTOR3)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_FORCE, HINA_GEOMETRY_ATTRIBUTE_TYPE_VECTOR3)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_FORCE_NORM, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_MASS, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_VOLUME, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_DENSITY, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_PRESSURE, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_NEIGHBOR_SUM_SELF, HINA_GEOMETRY_ATTRIBUTE_TYPE_INT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_NEIGHBOR_SUM_OTHERS, HINA_GEOMETRY_ATTRIBUTE_TYPE_INT)
}

void SIM_Hina_Particles::load()
{
	SIM_GeometryAutoReadLock lock(this);
	const GU_Detail *gdp = lock.getGdp();
	GA_ROHandleV3 pos_handle = gdp->getP();
	GA_ROHandleV3 vel_handle = gdp->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VELOCITY);
	GA_ROHandleV3 force_handle = gdp->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_FORCE);
	GA_ROHandleF mass_handle = gdp->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_MASS);
	GA_ROHandleF volume_handle = gdp->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VOLUME);
	GA_ROHandleF density_handle = gdp->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DENSITY);
	GA_Offset pt_off;
	GA_FOR_ALL_PTOFF(gdp, pt_off)
		{
			GA_Size index = gdp->pointIndex(pt_off);
			UT_Vector3 pos = pos_handle.get(pt_off);
			UT_Vector3 vel = vel_handle.get(pt_off);
			UT_Vector3 force = force_handle.get(pt_off);
			fpreal mass = mass_handle.get(pt_off);
			fpreal volume = volume_handle.get(pt_off);
			fpreal density = density_handle.get(pt_off);
			offset2index[pt_off] = index;
			index2offset[index] = pt_off;
			position_cache[pt_off] = pos;
			velocity_cache[pt_off] = vel;
			force_cache[pt_off] = force;
			mass_cache[pt_off] = mass;
			volume_cache[pt_off] = volume;
			density_cache[pt_off] = density;
		}
}
void SIM_Hina_Particles::commit()
{
	SIM_GeometryAutoWriteLock lock(this);
	GU_Detail &gdp = lock.getGdp();
	GA_RWHandleV3 pos_handle = gdp.getP();
	GA_RWHandleV3 vel_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VELOCITY);
	GA_RWHandleV3 force_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_FORCE);
	GA_RWHandleF force_n_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_FORCE_NORM);
	GA_RWHandleF mass_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_MASS);
	GA_RWHandleF volume_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VOLUME);
	GA_RWHandleF density_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DENSITY);
	GA_RWHandleI self_n_sum_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_NEIGHBOR_SUM_SELF);
	GA_RWHandleI other_n_sum_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_NEIGHBOR_SUM_OTHERS);
	GA_Offset pt_off;
	GA_FOR_ALL_PTOFF(&gdp, pt_off)
		{
			UT_Vector3 pos = position_cache[pt_off];
			UT_Vector3 vel = velocity_cache[pt_off];
			UT_Vector3 force = force_cache[pt_off];
			fpreal force_n = force.length();
			fpreal mass = mass_cache[pt_off];
			fpreal volume = volume_cache[pt_off];
			fpreal density = density_cache[pt_off];
			pos_handle.set(pt_off, pos);
			vel_handle.set(pt_off, vel);
			force_handle.set(pt_off, force);
			force_n_handle.set(pt_off, force_n);
			mass_handle.set(pt_off, mass);
			volume_handle.set(pt_off, volume);
			density_handle.set(pt_off, density);

			int fn_sum = neighbor_lists_cache[pt_off].size();
			int bn_sum = 0;
			for (auto &pair: other_neighbor_lists_cache)
				bn_sum += pair.second[pt_off].size();
			self_n_sum_handle.set(pt_off, fn_sum);
			other_n_sum_handle.set(pt_off, bn_sum);
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
		HinaPE::CubicSplineKernel<false> kernel(KernelRadius);

		for (size_t i = 0; i < points.Length(); ++i)
		{
			const Vector3D &point = points[i];
			double sum = 0.0;

			for (size_t j = 0; j < points.Length(); ++j)
			{
				const Vector3D &neighborPoint = points[j];
				sum += kernel.kernel(neighborPoint.DistanceTo(point));
			}

			maxNumberDensity = std::max(maxNumberDensity, sum);
		}

		this->UnivMass = TargetDensity / maxNumberDensity;
	}

	for (auto &m: mass_cache)
		m.second = this->UnivMass;
}
void SIM_Hina_Particles::calculate_volume() // deprecated
{
	HinaPE::CubicSplineKernel<false> kernel(getTargetSpacing() * getKernelRadiusOverTargetSpacing());
	for (auto &V: volume_cache)
	{
		GA_Offset pt_off = V.first;

		fpreal volume = 0.0;
		volume += kernel.kernel(0); // remember to include self (self is also a neighbor of itself)
		UT_Vector3 p_i = position_cache[pt_off];
		for_each_neighbor_self(pt_off, [&](const GA_Offset &n_off, const UT_Vector3 &n_pos)
		{
			UT_Vector3 p_j = n_pos;
			const UT_Vector3 r = p_i - p_j;
			volume += kernel.kernel(r.length());
		});
		for_each_neighbor_others(pt_off, [&](const GA_Offset &n_off, const UT_Vector3 &n_pos)
		{
			UT_Vector3 p_j = n_pos;
			const UT_Vector3 r = p_i - p_j;
			volume += kernel.kernel(r.length());
		});
		volume = 1.0 / volume;

		V.second = volume;
	}
}
void SIM_Hina_Particles::advect_position(fpreal dt)
{
	for_each_offset(
			[&](const GA_Offset &pt_off)
			{
				UT_Vector3 &pos = position_cache[pt_off];
				const UT_Vector3 &vel = velocity_cache[pt_off];
				pos += vel * dt;
			}
	);
}
void SIM_Hina_Particles::advect_velocity(fpreal dt)
{
	for_each_offset(
			[&](const GA_Offset &pt_off)
			{
				UT_Vector3 &vel = velocity_cache[pt_off];
				const UT_Vector3 &force = force_cache[pt_off];
				const fpreal mass = mass_cache[pt_off];
				vel += force / mass * dt;
			}
	);
}
void SIM_Hina_Particles::clear_force()
{
	for_each_offset(
			[&](GA_Offset pt_off)
			{
				force_cache[pt_off] = UT_Vector3(0, 0, 0);
			});
}
void SIM_Hina_Particles::calculate_force_gravity()
{
	for_each_offset(
			[&](GA_Offset pt_off)
			{
				force_cache[pt_off] += mass_cache[pt_off] * getGravity();
			});
}
size_t SIM_Hina_Particles::size() const
{
	return offset2index.size();
}
void SIM_Hina_Particles::for_each_offset(const std::function<void(const GA_Offset &)> &func)
{
	for (const auto &pair: offset2index)
		func(pair.first);
}
void SIM_Hina_Particles::for_each_neighbor_self(const GA_Offset &pt_off, const std::function<void(const GA_Offset &, const UT_Vector3 &)> &func)
{
	const auto &neighbors = neighbor_lists_cache[pt_off];
	for (const auto &neighbor: neighbors)
		func(neighbor.pt_off, neighbor.pt_pos);
}
void SIM_Hina_Particles::for_each_neighbor_others(const GA_Offset &pt_off, const std::function<void(const GA_Offset &, const UT_Vector3 &)> &func)
{
	for (auto &neighbor_lists: other_neighbor_lists_cache)
		for (const auto &neighbor: neighbor_lists.second[pt_off])
			func(neighbor.pt_off, neighbor.pt_pos);
}
void SIM_Hina_Particles::for_each_neighbor_others(const GA_Offset &pt_off, const std::function<void(const GA_Offset &, const UT_Vector3 &)> &func, const UT_String &other_name)
{
	const auto &neighbors = other_neighbor_lists_cache[other_name][pt_off];
	for (const auto &neighbor: neighbors)
		func(neighbor.pt_off, neighbor.pt_pos);
}
