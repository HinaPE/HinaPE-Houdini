#include "GAS_Hina_ClearForce.h"
#include <Particles/SIM_Hina_Particles.h>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		ClearForce,
		true,
		false,
		TARGET_PARTICLE_GEOMETRY(SIM_Hina_Particles)
)

void GAS_Hina_ClearForce::_init() {}
void GAS_Hina_ClearForce::_makeEqual(const GAS_Hina_ClearForce *src) {}
bool GAS_Hina_ClearForce::_solve(SIM_Engine &, SIM_Object *obj, SIM_Time, SIM_Time)
{
	SIM_Hina_Particles *particles = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_Particles);
	CHECK_NULL_RETURN_BOOL(particles)

	SIM_GeometryAutoWriteLock lock(particles);
	GU_Detail &gdp = lock.getGdp();
	GA_RWHandleV3 force_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_FORCE);
	GA_Offset pt_off;
	GA_FOR_ALL_PTOFF(&gdp, pt_off) { force_handle.set(pt_off, UT_Vector3(0.)); }
	return true;
}
