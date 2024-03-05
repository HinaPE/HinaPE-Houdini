#include "GAS_Hina_Solver_DFSPH.h"

GAS_HINA_SUBSOLVER_IMPLEMENT(
		Solver_DFSPH,
		true,
		false,
		HINA_FLOAT_VECTOR_PARAMETER(FluidDomain, 3, 1., 1., 1.) \
        HINA_BOOL_PARAMETER(TopOpen, true) \
        HINA_FLOAT_PARAMETER(TargetSpacing, .02) \
        HINA_FLOAT_PARAMETER(KernelRadius, .04) \
        HINA_FLOAT_PARAMETER(TargetDensity, 1000.) \
        HINA_FLOAT_VECTOR_PARAMETER(Gravity, 3, 0, -9.8, 0) \
        HINA_INT_PARAMETER(MaxNumOfParticles, 100000) \
        HINA_BOOL_PARAMETER(IsOneShot, true) \
        HINA_FLOAT_VECTOR_PARAMETER(EmitStart, 3, -.3, -.3, -.3) \
        HINA_FLOAT_VECTOR_PARAMETER(EmitEnd, 3, .3, .3, .3) \
        static std::array<PRM_Name, 5> BoundaryHandling = {\
            PRM_Name("0", "Akinci2012"), \
            PRM_Name("1", "Koschier2017"), \
            PRM_Name("2", "Bender2019"), \
            PRM_Name("3", "None"), \
            PRM_Name(nullptr),}; \
        static PRM_Name BoundaryHandlingName("BoundaryHandling", "BoundaryHandling"); \
        static PRM_Default BoundaryHandlingNameDefault(0, "Akinci2012"); \
        static PRM_ChoiceList CLBoundaryHandling(PRM_CHOICELIST_SINGLE, BoundaryHandling.data()); \
        PRMS.emplace_back(PRM_ORD, 1, &BoundaryHandlingName, &BoundaryHandlingNameDefault, &CLBoundaryHandling); \
        static std::array<PRM_Name, 4> Kernels = {\
            PRM_Name("0", "Poly64"), \
            PRM_Name("1", "Spiky"), \
            PRM_Name("2", "CubicSpline"), \
            PRM_Name(nullptr),}; \
        static PRM_Name KernelName("Kernel", "Kernel"); \
        static PRM_Default KernelNameDefault(2, "CubicSpline"); \
        static PRM_ChoiceList CLKernel(PRM_CHOICELIST_SINGLE, Kernels.data()); \
        PRMS.emplace_back(PRM_ORD, 1, &KernelName, &KernelNameDefault, &CLKernel); \
        TARGET_PARTICLE_GEOMETRY(SIM_Hina_Particles_DFSPH)
)
void GAS_Hina_Solver_DFSPH::_init()
{
	this->DFSPH_AkinciSolverPtr = nullptr;
	this->inited = false;
	this->emitted = false;
}
void GAS_Hina_Solver_DFSPH::_makeEqual(const GAS_Hina_Solver_DFSPH *src)
{
	this->DFSPH_AkinciSolverPtr = src->DFSPH_AkinciSolverPtr;
	this->inited = src->inited;
	this->emitted = src->emitted;
}
bool GAS_Hina_Solver_DFSPH::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_Hina_Particles_DFSPH *DFSPH_particles = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_Particles_DFSPH);
	if (DFSPH_particles == nullptr)
		return true;

	if (!inited)
		init_data(DFSPH_particles, obj);

	if (!emitted)
		emit_data(DFSPH_particles);

	if (DFSPH_AkinciSolverPtr)
	{
		UpdateAllAkinciBoundaries(obj);
		DFSPH_AkinciSolverPtr->Solve(timestep);
		apply_akinci_force(obj);
	}

	return true;
}

/// mapping pointers to the solver, and set solvers' parameters (SHOULD BE DONE AT FIRST STEP)
void GAS_Hina_Solver_DFSPH::init_data(SIM_Hina_Particles_DFSPH *DFSPH_particles, SIM_Object *obj)
{
	switch (getBoundaryHandling())
	{
		case 0: // Akinci2012
		{
			DFSPH_AkinciSolverPtr = std::make_shared<HinaPE::DFSPH_AkinciSolver>(static_cast<real>(getKernelRadius()), getFluidDomainF());
			DFSPH_AkinciSolverPtr->FLUID_REST_DENSITY = getTargetDensity();
			DFSPH_AkinciSolverPtr->FLUID_PARTICLE_RADIUS = getTargetSpacing() / 2.;
			DFSPH_AkinciSolverPtr->GRAVITY = getGravityF();
			DFSPH_AkinciSolverPtr->TOP_OPEN = getTopOpen();

			DFSPH_particles->x = &DFSPH_AkinciSolverPtr->Fluid->x;
			DFSPH_particles->v = &DFSPH_AkinciSolverPtr->Fluid->v;
			DFSPH_particles->a = &DFSPH_AkinciSolverPtr->Fluid->a;
			DFSPH_particles->m = &DFSPH_AkinciSolverPtr->Fluid->m;
			DFSPH_particles->V = &DFSPH_AkinciSolverPtr->Fluid->V;
			DFSPH_particles->rho = &DFSPH_AkinciSolverPtr->Fluid->rho;
			DFSPH_particles->nt = &DFSPH_AkinciSolverPtr->Fluid->neighbor_this;
			DFSPH_particles->no = &DFSPH_AkinciSolverPtr->Fluid->neighbor_others;
			DFSPH_particles->factor = &DFSPH_AkinciSolverPtr->Fluid->factor;
			DFSPH_particles->k = &DFSPH_AkinciSolverPtr->Fluid->k;
			DFSPH_particles->density_adv = &DFSPH_AkinciSolverPtr->Fluid->density_adv;

			std::vector<SIM_Hina_Particles_Akinci *> akinci_boundaries = FetchAllAkinciBoundaries(obj);
			for (auto &akinci_boundary: akinci_boundaries)
			{
				DFSPH_AkinciSolverPtr->Boundaries.emplace_back(std::make_shared<HinaPE::AkinciBoundary>());
				akinci_boundary->x = &DFSPH_AkinciSolverPtr->Boundaries.back()->x;
				akinci_boundary->v = &DFSPH_AkinciSolverPtr->Boundaries.back()->v;
				akinci_boundary->a = &DFSPH_AkinciSolverPtr->Boundaries.back()->a;
				akinci_boundary->m = &DFSPH_AkinciSolverPtr->Boundaries.back()->m;
				akinci_boundary->V = &DFSPH_AkinciSolverPtr->Boundaries.back()->V;
				akinci_boundary->rho = &DFSPH_AkinciSolverPtr->Boundaries.back()->rho;
				akinci_boundary->nt = &DFSPH_AkinciSolverPtr->Boundaries.back()->neighbor_this;
				akinci_boundary->no = &DFSPH_AkinciSolverPtr->Boundaries.back()->neighbor_others;
				akinci_boundary->x_init = &DFSPH_AkinciSolverPtr->Boundaries.back()->x_init;
				akinci_boundary->xform = &DFSPH_AkinciSolverPtr->Boundaries.back()->xform;

				akinci_boundary->b_set_index = DFSPH_AkinciSolverPtr->Boundaries.size() - 1;

				DFSPH_AkinciSolverPtr->BOUNDARY_REST_DENSITY.emplace_back(static_cast<real>(akinci_boundary->getSolidDensity()));
				DFSPH_AkinciSolverPtr->BOUNDARY_DYNAMICS.emplace_back(akinci_boundary->getIsDynamic());
			}
		}
			break;
		default:
			break;
	}

	InitAllAkinciBoundaries(obj); // load from sop into x_init
	inited = true;
}

/// Emit particles (with any method)
void GAS_Hina_Solver_DFSPH::emit_data(SIM_Hina_Particles_DFSPH *DFSPH_particles)
{
	size_t max_p = getMaxNumOfParticles();
	bool one_shot = getIsOneShot();
	real spacing = getTargetSpacing();
	Vector start = getEmitStart();
	Vector end = getEmitEnd();

	HinaPE::FluidEmitter::UseFluidBlock(DFSPH_particles->x, start, end, spacing);

	emitted = one_shot;
}

void GAS_Hina_Solver_DFSPH::apply_akinci_force(SIM_Object *obj)
{
	for (auto &rigidbody: FetchAllRigidBodies(obj))
	{
		rigidbody->rb->resetForce();
		rigidbody->rb->resetTorque();
		for (auto &boundary: FetchAllAkinciBoundaries(obj))
		{
			if (rigidbody->b_set_index == -1 || boundary->b_set_index == -1)
				continue;

			if (boundary->b_set_index == rigidbody->b_set_index)
			{
				for (int i = 0; i < boundary->a->size(); ++i)
				{
					real buoyancy = boundary->getBuoyancy();
					Vector _f = (*boundary->m)[i] * (*boundary->a)[i];
					_f *= buoyancy;
					Vector _x = (*boundary->x)[i];

					reactphysics3d::Vector3 f = {_f.x(), _f.y(), _f.z()};
					reactphysics3d::Vector3 x = {_x.x(), _x.y(), _x.z()};
					rigidbody->rb->applyWorldForceAtWorldPosition(f, x);
				}
			}
		}

		rigidbody->rb->setAngularDamping(0.5);
		rigidbody->rb->resetTorque();
	}
}
