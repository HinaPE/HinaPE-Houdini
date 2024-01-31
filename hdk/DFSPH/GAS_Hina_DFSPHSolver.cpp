#include "GAS_Hina_DFSPHSolver.h"
#include <DFSPH/SIM_Hina_DFSPHParticles.h>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		DFSPHSolver,
		true,
		false,
		TARGET_PARTICLE_GEOMETRY(SIM_Hina_DFSPHParticles)
)

void GAS_Hina_DFSPHSolver::_init() {}
void GAS_Hina_DFSPHSolver::_makeEqual(const GAS_Hina_DFSPHSolver *src) {}
bool GAS_Hina_DFSPHSolver::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_Hina_DFSPHParticles *particles = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_DFSPHParticles);
	CHECK_NULL_RETURN_BOOL(particles)

	SIM_GeometryAutoWriteLock lock(particles);
	GU_Detail &gdp = lock.getGdp();

	return true;
}
