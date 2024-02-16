#include "GAS_Hina_AdvectPos.h"
#include <Particles/SIM_Hina_Particles.h>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		AdvectPos,
		true,
		false,
		TARGET_PARTICLE_GEOMETRY(SIM_Hina_Particles)
)
void GAS_Hina_AdvectPos::_init() {}
void GAS_Hina_AdvectPos::_makeEqual(const GAS_Hina_AdvectPos *src) {}
bool GAS_Hina_AdvectPos::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_Hina_Particles *particles = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_Particles);
	CHECK_NULL_RETURN_BOOL(particles)

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

			pos = pos + dt * vel;
			particles->positions_cache[pt_off] = pos;
		}
	return true;
}
