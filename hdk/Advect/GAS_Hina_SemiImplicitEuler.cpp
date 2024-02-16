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

	SIM_GeometryAutoReadLock lock(particles);
	const GU_Detail *gdp = lock.getGdp();
	GA_ROHandleV3 pos_handle = gdp->getP();
	GA_ROHandleV3 vel_handle = gdp->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VELOCITY);
	GA_ROHandleV3 force_handle = gdp->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_FORCE);
	GA_ROHandleF mass_handle = gdp->findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_MASS);
	double dt = timestep;
	GA_Offset pt_off;
	GA_FOR_ALL_PTOFF(gdp, pt_off)
		{
			UT_Vector3 pos = pos_handle.get(pt_off);
			UT_Vector3 vel = vel_handle.get(pt_off);
			UT_Vector3 force = force_handle.get(pt_off);
			fpreal mass = mass_handle.get(pt_off);

			vel = vel + dt * force / mass;
			pos = pos + dt * vel;
			particles->positions_cache[pt_off] = pos;
			particles->velocity_cache[pt_off] = vel;
		}
	return true;
}
