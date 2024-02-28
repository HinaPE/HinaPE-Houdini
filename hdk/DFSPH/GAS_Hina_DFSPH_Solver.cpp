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

	HinaPE::real kernel_radius = DFSPH_particles->getTargetSpacing() * DFSPH_particles->getKernelRadiusOverTargetSpacing();

	if (!inited)
	{
		SolverPtr = std::make_shared<HinaPE::DFSPHSolverCPU>(kernel_radius);

		DFSPH_particles->x = &SolverPtr->Fluid.x;
		DFSPH_particles->v = &SolverPtr->Fluid.v;
		DFSPH_particles->f = &SolverPtr->Fluid.f;
		DFSPH_particles->m = &SolverPtr->Fluid.m;
		DFSPH_particles->V = &SolverPtr->Fluid.V;
		DFSPH_particles->rho = &SolverPtr->Fluid.rho;
		DFSPH_particles->neighbor_this = &SolverPtr->Fluid.neighbor_this;
		DFSPH_particles->neighbor_others = &SolverPtr->Fluid.neighbor_others;

		DFSPH_particles->alpha = &SolverPtr->Fluid.alpha;
		DFSPH_particles->kappa_density = &SolverPtr->Fluid.kappa_density;
		DFSPH_particles->kappa_divergence = &SolverPtr->Fluid.kappa_divergence;
		DFSPH_particles->rho_adv = &SolverPtr->Fluid.rho_adv;
		DFSPH_particles->d_rho = &SolverPtr->Fluid.d_rho;

		DFSPH_particles->load();

		std::map<UT_String, SIM_Hina_Akinci_Particles *> akinci_boundaries = FetchAllAkinciBoundariesAndApply(obj, [&](SIM_Object *obj_boundary, SIM_Hina_Akinci_Particles *akinci_boundary, const UT_String &boundary_name)
		{
			akinci_boundary->load_sop(obj_boundary);
		});

		SolverPtr->StaticBoundaries.resize(akinci_boundaries.size());
		int iter = 0;
		for (auto &akinci_boundary: akinci_boundaries)
		{
			akinci_boundary.second->x = &(SolverPtr->StaticBoundaries[iter].x);
			akinci_boundary.second->v = &(SolverPtr->StaticBoundaries[iter].v);
			akinci_boundary.second->f = &(SolverPtr->StaticBoundaries[iter].f);
			akinci_boundary.second->m = &(SolverPtr->StaticBoundaries[iter].m);
			akinci_boundary.second->V = &(SolverPtr->StaticBoundaries[iter].V);
			akinci_boundary.second->rho = &(SolverPtr->StaticBoundaries[iter].rho);
			akinci_boundary.second->neighbor_this = &(SolverPtr->StaticBoundaries[iter].neighbor_this);
			akinci_boundary.second->neighbor_others = &(SolverPtr->StaticBoundaries[iter].neighbor_others);

			akinci_boundary.second->load();

			++iter;
		}

		SolverPtr->Init();
		SolverPtr->bound = DFSPH_particles->getFluidDomain().x(); // TODO: remove here

		for (auto &akinci_boundary: akinci_boundaries)
			akinci_boundary.second->commit();
		inited = true;
	}
	DFSPH_particles->load(); // update fluid data if dirty
	SolverPtr->Solve(timestep);
	return true;
}
