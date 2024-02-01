#include "SIM_Hina_Particles.h"
#include <Collision/SIM_Hina_RigidBodyCollider.h>

#include <Common/CUDA_CubbyFlow/Core/Geometry/BoundingBox.hpp>
#include <Common/CUDA_CubbyFlow/Core/Particle/SPHKernels.hpp>
#include <Common/CUDA_CubbyFlow/Core/Particle/SPHSystemData.hpp>
#include <Common/CUDA_CubbyFlow/Core/PointGenerator/BccLatticePointGenerator.hpp>
#include <Common/CUDA_CubbyFlow/Core/PointGenerator/TrianglePointGenerator.hpp>

SIM_HINA_GEOMETRY_IMPLEMENT(
		Particles,
		false,
		HINA_FLOAT_VECTOR_PARAMETER(FluidDomain, 3, 2, 2, 2) \
        HINA_FLOAT_PARAMETER(TargetSpacing, .02) \
        HINA_FLOAT_PARAMETER(KernelRadiusOverTargetSpacing, 1.8) \
        HINA_FLOAT_PARAMETER(TargetDensity, 1000.) \
)
void SIM_Hina_Particles::_init_Particles()
{
	this->Mass = 1;
	this->neighbor_lists_cache.clear();
	this->positions_cache.clear();
	this->velocity_cache.clear();
}
void SIM_Hina_Particles::_makeEqual_Particles(const SIM_Hina_Particles *src)
{
	this->Mass = src->Mass;
	this->neighbor_lists_cache = src->neighbor_lists_cache;
	this->positions_cache = src->positions_cache;
	this->velocity_cache = src->velocity_cache;
}
void SIM_Hina_Particles::_setup_gdp(GU_Detail *gdp) const
{
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_COLOR, HINA_GEOMETRY_ATTRIBUTE_TYPE_VECTOR3)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_VELOCITY, HINA_GEOMETRY_ATTRIBUTE_TYPE_VECTOR3)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_FORCE, HINA_GEOMETRY_ATTRIBUTE_TYPE_VECTOR3)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_MASS, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_DENSITY, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_PRESSURE, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_NEIGHBOR_SUM, HINA_GEOMETRY_ATTRIBUTE_TYPE_INT)

	fpreal TargetSpacing = getTargetSpacing();
	fpreal TargetDensity = getTargetDensity();
	fpreal KernelRadius = getKernelRadiusOverTargetSpacing() * getTargetSpacing();
	// Compute Mass
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
		SPHStdKernel3 kernel{KernelRadius};

		for (size_t i = 0; i < points.Length(); ++i)
		{
			const Vector3D &point = points[i];
			double sum = 0.0;

			for (size_t j = 0; j < points.Length(); ++j)
			{
				const Vector3D &neighborPoint = points[j];
				sum += kernel(neighborPoint.DistanceTo(point));
			}

			maxNumberDensity = std::max(maxNumberDensity, sum);
		}

		this->Mass = TargetDensity / maxNumberDensity;
	}
}
void SIM_Hina_Particles::Commit()
{
	SIM_GeometryAutoWriteLock lock(this);
	GU_Detail &gdp = lock.getGdp();
	GA_RWHandleV3 pos_handle = gdp.getP();
	GA_RWHandleV3 vel_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VELOCITY);
	GA_RWHandleI n_sum_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_NEIGHBOR_SUM);
	GA_RWHandleV3 cd_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_COLOR);
	GA_Offset pt_off;
	GA_FOR_ALL_PTOFF(&gdp, pt_off)
		{
			UT_Vector3 pos = positions_cache[pt_off];
			UT_Vector3 vel = velocity_cache[pt_off];
			int n_sum = neighbor_lists_cache[pt_off].size();
			pos_handle.set(pt_off, pos);
			vel_handle.set(pt_off, vel);
			n_sum_handle.set(pt_off, n_sum);

			fpreal v_l = vel.length();
			fpreal c = std::clamp(v_l / 10, 0., 1.);
			cd_handle.set(pt_off, UT_Vector3(c, c, 1));
		}
}

GAS_HINA_SUBSOLVER_IMPLEMENT(
		CommitCache,
		true,
		false,
		TARGET_PARTICLE_GEOMETRY(SIM_Hina_Particles)
)
void GAS_Hina_CommitCache::_init() {}
void GAS_Hina_CommitCache::_makeEqual(const GAS_Hina_CommitCache *src) {}
bool GAS_Hina_CommitCache::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_Hina_Particles *particles = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_Particles);
	CHECK_NULL_RETURN_BOOL(particles)
	particles->Commit();
	return true;
}
SIM_Guide *GAS_Hina_CommitCache::createGuideObjectSubclass() const { return new SIM_GuideShared(this, true); }
void GAS_Hina_CommitCache::buildGuideGeometrySubclass(const SIM_RootData &root, const SIM_Options &options, const GU_DetailHandle &gdh, UT_DMatrix4 *xform, const SIM_Time &t) const
{
	if (gdh.isNull())
		return;

	if (!getShowGuideGeometry(options))
		return;

	UT_Vector3 color = getDomainColor(options);

	GU_DetailHandleAutoWriteLock gdl(gdh);
	GU_Detail *gdp = gdl.getGdp();
	gdp->clearAndDestroy();

	UT_Vector3 Center = UT_Vector3(0.);
	UT_Vector3 Extent = {2, 2, 2};
//	getFluidDomain();

	std::array<UT_Vector3, 8> vertices{};
	for (int i = 0; i < 8; i++)
	{
		vertices[i] = UT_Vector3(
				Center.x() + Extent.x() * ((i & 1) ? 0.5 : -0.5),
				Center.y() + Extent.y() * ((i & 2) ? 0.5 : -0.5),
				Center.z() + Extent.z() * ((i & 4) ? 0.5 : -0.5)
		);
	}

	std::array<GA_Offset, 8> pt_off{};
	for (int i = 0; i < 8; i++)
	{
		pt_off[i] = gdp->appendPointOffset();
		gdp->setPos3(pt_off[i], vertices[i]);

		GA_RWHandleV3 gdp_handle_cd(gdp->addFloatTuple(GA_ATTRIB_POINT, "Cd", 3));
		gdp_handle_cd.set(pt_off[i], color);
	}

	static const int edges[12][2] = {
			{0, 1},
			{0, 4},
			{1, 3},
			{1, 5},
			{2, 0},
			{2, 3},
			{2, 6},
			{3, 7},
			{4, 5},
			{4, 6},
			{5, 7},
			{6, 7},
	};

	for (int i = 0; i < 12; i++)
	{
		GEO_PrimPoly *line = GEO_PrimPoly::build(gdp, 2, GU_POLY_OPEN);
		for (int j = 0; j < 2; j++)
			line->setVertexPoint(j, pt_off[edges[i][j]]);
	}
}
