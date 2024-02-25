#include "GAS_Hina_GravityForce.h"
#include <Particles/SIM_Hina_Particles.h>

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

	calculate_gravity_force(particles);
	return true;
}
void GAS_Hina_GravityForce::calculate_gravity_force(SIM_Hina_Particles *particles)
{
	particles->for_each_offset(
			[&](GA_Offset pt_off)
			{
				particles->force_cache[pt_off] += particles->mass_cache[pt_off] * getGravity();
			});
}
