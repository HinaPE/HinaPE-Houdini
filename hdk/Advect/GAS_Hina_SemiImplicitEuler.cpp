#include "GAS_Hina_SemiImplicitEuler.h"
#include <Particles/SIM_Hina_Particles.h>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		SemiImplicitEuler,
		true,
		false,
		TARGET_PARTICLE_GEOMETRY(SIM_Hina_Particles)
)

void GAS_Hina_SemiImplicitEuler::_init() {}
void GAS_Hina_SemiImplicitEuler::_makeEqual(const GAS_Hina_SemiImplicitEuler *src) {}
bool GAS_Hina_SemiImplicitEuler::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_Hina_Particles *particles = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_Particles);
	CHECK_NULL_RETURN_BOOL(particles)

	particles->for_each_offset(
			[&](GA_Offset pt_off)
			{
				UT_Vector3 pos = particles->position_cache[pt_off];
				UT_Vector3 vel = particles->velocity_cache[pt_off];
				UT_Vector3 force = particles->force_cache[pt_off];
				fpreal mass = particles->mass_cache[pt_off];

				vel = vel + timestep * force / mass;
				pos = pos + timestep * vel;
				particles->position_cache[pt_off] = pos;
				particles->velocity_cache[pt_off] = vel;
			});
	return true;
}
