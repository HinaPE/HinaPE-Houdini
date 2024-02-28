#include "GAS_Hina_DFSPH_SolverSPlisHSPlasH.h"
#include <DFSPH/SIM_Hina_DFSPH_Particles.h>
#include <Akinci2012/SIM_Hina_Akinci_Particles.h>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		DFSPH_SolverSPlisHSPlasH,
		true,
		false,
		TARGET_PARTICLE_GEOMETRY(SIM_Hina_DFSPH_Particles)
)
void GAS_Hina_DFSPH_SolverSPlisHSPlasH::_init()
{
	this->SolverPtr = nullptr;
	this->inited = false;
}
void GAS_Hina_DFSPH_SolverSPlisHSPlasH::_makeEqual(const GAS_Hina_DFSPH_SolverSPlisHSPlasH *src)
{
	this->SolverPtr = src->SolverPtr;
	this->inited = src->inited;
}
bool GAS_Hina_DFSPH_SolverSPlisHSPlasH::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_Hina_DFSPH_Particles *DFSPH_particles = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_DFSPH_Particles);
	CHECK_NULL_RETURN_BOOL(DFSPH_particles)

	HinaPE::real kernel_radius = DFSPH_particles->getTargetSpacing() * DFSPH_particles->getKernelRadiusOverTargetSpacing();

}
