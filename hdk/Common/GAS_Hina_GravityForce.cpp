#include "GAS_Hina_GravityForce.h"
#include <Common/SIM_Hina_Particles.h>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		GravityForce,
		true,
		false,
		TARGET_PARTICLE_GEOMETRY(SIM_Hina_Particles)
		HINA_FLOAT_VECTOR_PARAMETER(Gravity, 3, 0, -9.8, 0)
)

void GAS_Hina_GravityForce::_init() {}
void GAS_Hina_GravityForce::_makeEqual(const GAS_Hina_GravityForce *src) {}
bool GAS_Hina_GravityForce::_solve(SIM_Engine &, SIM_Object *obj, SIM_Time, SIM_Time)
{
	SIM_Hina_Particles *particles = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_Particles);
	CHECK_NULL_RETURN_BOOL(particles)

	SIM_GeometryAutoWriteLock lock(particles);
	GU_Detail &gdp = lock.getGdp();
	GA_RWHandleV3 force_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_FORCE);
	GA_RWHandleF mass_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_MASS);
	UT_Vector3 gravity = getGravity();
	GA_Offset pt_off;
	GA_FOR_ALL_PTOFF(&gdp, pt_off)
		{
			UT_Vector3 origin = force_handle.get(pt_off);
			fpreal mass = mass_handle.get(pt_off);
			force_handle.set(pt_off, origin + mass * gravity);
		}
	return true;
}
