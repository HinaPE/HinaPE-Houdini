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

	calculate_clear_force(particles);
	return true;
}
void GAS_Hina_ClearForce::calculate_clear_force(SIM_Hina_Particles *particles)
{
	particles->for_each_offset(
			[&](GA_Offset pt_off)
			{
				particles->force_cache[pt_off] = UT_Vector3(0, 0, 0);
			});
}
