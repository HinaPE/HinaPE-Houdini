#include "GAS_Hina_EnforceBoundary.h"
#include <Particles/SIM_Hina_Particles.h>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		EnforceBoundary,
		true,
		false,
		TARGET_PARTICLE_GEOMETRY(SIM_Hina_Particles)
)
void GAS_Hina_EnforceBoundary::_init() {}
void GAS_Hina_EnforceBoundary::_makeEqual(const GAS_Hina_EnforceBoundary *src) {}
bool GAS_Hina_EnforceBoundary::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_Hina_Particles *particles = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_Particles);
	CHECK_NULL_RETURN_BOOL(particles)

	UT_Vector3 Domain =  particles->getFluidDomain();
	UT_Vector3 HalfDomain = Domain / 2;
	HalfDomain *= 0.999f; // avoid corner case

	SIM_GeometryAutoReadLock lock(particles);
	const GU_Detail *gdp = lock.getGdp();
	GA_ROHandleV3 pos_handle = gdp->getP();
	GA_ROHandleV3 vel_handle = gdp->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VELOCITY);
	double dt = timestep;
	GA_Offset pt_off;
	GA_FOR_ALL_PTOFF(gdp, pt_off)
		{
			UT_Vector3 pos = pos_handle.get(pt_off);
			UT_Vector3 vel = vel_handle.get(pt_off);

			if (pos.x() <= -HalfDomain.x()) { pos.x() = -HalfDomain.x(); vel.x() = std::max(0.f, vel.x()); }
			if (pos.x() >= HalfDomain.x()) { pos.x() = HalfDomain.x(); vel.x() = std::min(0.f, vel.x()); }
			if (pos.y() <= -HalfDomain.y()) { pos.y() = -HalfDomain.y(); vel.y() = std::max(0.f, vel.y()); }
			if (pos.y() >= HalfDomain.y()) { pos.y() = HalfDomain.y(); vel.y() = std::min(0.f, vel.y()); }
			if (pos.z() <= -HalfDomain.z()) { pos.z() = -HalfDomain.z(); vel.z() = std::max(0.f, vel.z()); }
			if (pos.z() >= HalfDomain.z()) { pos.z() = HalfDomain.z(); vel.z() = std::min(0.f, vel.z()); }

			particles->positions_cache[pt_off] = pos;
			particles->velocity_cache[pt_off] = vel;
		}
	return true;
}
