#include "GAS_Hina_DFSPH_Solver.h"
#include <DFSPH/SIM_Hina_DFSPH_Particles.h>
#include <Akinci2012/SIM_Hina_Akinci_Particles.h>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		DFSPH_Solver,
		true,
		false,
		TARGET_PARTICLE_GEOMETRY(SIM_Hina_DFSPH_Particles)
)
void GAS_Hina_DFSPH_Solver::_init()
{
	this->SolverPtr = nullptr;
	this->inited = false;
}
void GAS_Hina_DFSPH_Solver::_makeEqual(const GAS_Hina_DFSPH_Solver *src)
{
	this->SolverPtr = src->SolverPtr;
	this->inited = src->inited;
}
bool GAS_Hina_DFSPH_Solver::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_Hina_DFSPH_Particles *DFSPH_particles = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_DFSPH_Particles);
	CHECK_NULL_RETURN_BOOL(DFSPH_particles)

	DFSPH_particles->x = &SolverPtr->Fluid.x;
	DFSPH_particles->v = &SolverPtr->Fluid.v;
	DFSPH_particles->f = &SolverPtr->Fluid.f;
	DFSPH_particles->m = &SolverPtr->Fluid.m;
	DFSPH_particles->V = &SolverPtr->Fluid.V;
	DFSPH_particles->rho = &SolverPtr->Fluid.rho;

	DFSPH_particles->alpha = &SolverPtr->Fluid.alpha;
	DFSPH_particles->kappa_density = &SolverPtr->Fluid.kappa_density;
	DFSPH_particles->kappa_divergence = &SolverPtr->Fluid.kappa_divergence;
	DFSPH_particles->rho_adv = &SolverPtr->Fluid.rho_adv;
	DFSPH_particles->d_rho = &SolverPtr->Fluid.d_rho;

	DFSPH_particles->load();

	if (!inited)
	{
		SolverPtr->Init(*DFSPH_particles->x, *DFSPH_particles->x);
		inited = true;
	}
	SolverPtr->Solve(timestep);
	return true;
}
