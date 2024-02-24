#include "GAS_Hina_CollisionSolve.h"
#include <Particles/SIM_Hina_Particles.h>
#include <Collision/SIM_Hina_RigidBodyCollider.h>

#include <CUDA_CubbyFlow/Core/Geometry/RigidBodyCollider.hpp>
#include <CUDA_CubbyFlow/Core/Geometry/Box.hpp>
#include <CUDA_CubbyFlow/Core/Array/Array.hpp>
#include <CUDA_CubbyFlow/Core/Utils/Parallel.hpp>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		CollisionSolve,
		true,
		false,
		TARGET_PARTICLE_GEOMETRY(SIM_Hina_Particles)
)

void GAS_Hina_CollisionSolve::_init() {}
void GAS_Hina_CollisionSolve::_makeEqual(const GAS_Hina_CollisionSolve *src) {}
bool GAS_Hina_CollisionSolve::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_Hina_Particles *particles = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_Particles);
	CHECK_NULL_RETURN_BOOL(particles)

	size_t pt_size = particles->getNumPoints();
	fpreal pt_radius = particles->getTargetSpacing();

	CubbyFlow::Array1<CubbyFlow::Vector3D> pos_array;
	CubbyFlow::Array1<CubbyFlow::Vector3D> vel_array;
	{
		SIM_GeometryAutoWriteLock lock(particles);
		GU_Detail &gdp = lock.getGdp();
		pos_array.Resize(pt_size);
		vel_array.Resize(pt_size);
		for (int idx = 0; idx < pt_size; ++idx)
		{
			GA_Offset pt_off = gdp.pointOffset(idx);
			pos_array[idx] = AS_CFVector3D(particles->position_cache[pt_off]);
			vel_array[idx] = AS_CFVector3D(particles->velocity_cache[pt_off]);
		}
	}

	// Fluid Domain Collider
	SIM_Hina_RigidBodyCollider *fluid_domain_collider_data = SIM_DATA_CREATE(*obj, SIM_Hina_RigidBodyCollider::DATANAME, SIM_Hina_RigidBodyCollider,
																			 SIM_DATA_RETURN_EXISTING |
																			 SIM_DATA_ADOPT_EXISTING_ON_DELETE);
	if (!fluid_domain_collider_data->Configured)
	{
		UT_Vector3D MaxRegion = particles->getFluidDomainD();
		CubbyFlow::BoundingBox3D fluid_domain(
				CubbyFlow::Vector3D(-MaxRegion.x() / 2, -MaxRegion.y() / 2, -MaxRegion.z() / 2),
				CubbyFlow::Vector3D(MaxRegion.x() / 2, MaxRegion.y() / 2, MaxRegion.z() / 2)
		);
		const auto box = CubbyFlow::Box3::GetBuilder()
				.WithBoundingBox(fluid_domain)
				.WithIsNormalFlipped(true)
				.MakeShared();
		fluid_domain_collider_data->InnerPtr = CubbyFlow::RigidBodyCollider3::GetBuilder()
				.WithSurface(box)
				.MakeShared();
		fluid_domain_collider_data->Configured = true;
	}

	SIM_ObjectArray affectors;
	obj->getAffectors(affectors, "SIM_RelationshipCollide");
	exint num_affectors = affectors.entries();
	for (int i = 0; i < num_affectors; ++i)
	{
		SIM_Object *affector = affectors(i);
		SIM_Hina_RigidBodyCollider *col = SIM_DATA_GET(*affector, SIM_Hina_RigidBodyCollider::DATANAME, SIM_Hina_RigidBodyCollider);
		if (!col)
			continue;
		if (!col->Configured)
			continue;
		fpreal restitution = col->getRestitutionCoefficient();
		col->InnerPtr->Update(time, timestep); // TODO: fix time here
		CubbyFlow::ParallelFor(CubbyFlow::ZERO_SIZE, pt_size, [&](size_t i)
		{
			col->InnerPtr->ResolveCollision(pt_radius, restitution, &pos_array[i], &vel_array[i]);
		});
	}

	{
		SIM_GeometryAutoWriteLock lock(particles);
		GU_Detail &gdp = lock.getGdp();
		for (int idx = 0; idx < pt_size; ++idx)
		{
			GA_Offset pt_off = gdp.pointOffset(idx);
			particles->position_cache[pt_off] = AS_UTVector3D(pos_array[idx]);
			particles->velocity_cache[pt_off] = AS_UTVector3D(vel_array[idx]);
		}
	}

	return true;
}
