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

	particles->advect_velocity(timestep);
	particles->advect_position(timestep);
	return true;
}
