#include "GAS_Hina_Solver_PBF.h"

GAS_HINA_SUBSOLVER_IMPLEMENT(
		Solver_PBF,
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
        TARGET_PARTICLE_GEOMETRY(SIM_Hina_Particles_PBF)
)

void GAS_Hina_Solver_PBF::_init()
{
	this->PBF_AkinciSolverPtr = nullptr;
	this->inited = false;
	this->emitted = false;
}
void GAS_Hina_Solver_PBF::_makeEqual(const GAS_Hina_Solver_PBF *src)
{
	this->PBF_AkinciSolverPtr = src->PBF_AkinciSolverPtr;
	this->inited = src->inited;
	this->emitted = src->emitted;
}
bool GAS_Hina_Solver_PBF::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_Hina_Particles_PBF *PBF_particles = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_Particles_PBF);
	if (PBF_particles == nullptr)
		return true;

	if (!inited)
		init_data(PBF_particles, obj);

	if (!emitted)
		emit_data(PBF_particles);

	if (PBF_AkinciSolverPtr)
	{
		PBF_AkinciSolverPtr->Solve(timestep);
	}

	return true;
}

void GAS_Hina_Solver_PBF::init_data(SIM_Hina_Particles_PBF *PBF_particles, SIM_Object *obj)
{
	switch (getBoundaryHandling())
	{
		case 0: // Akinci2012
		{
			PBF_AkinciSolverPtr = std::make_shared<HinaPE::PBF_AkinciSolver>(static_cast<real>(getKernelRadius()), getFluidDomainF());
			PBF_AkinciSolverPtr->FLUID_REST_DENSITY = getTargetDensity();
			PBF_AkinciSolverPtr->FLUID_PARTICLE_RADIUS = getTargetSpacing() / 2.;
			PBF_AkinciSolverPtr->GRAVITY = getGravityF();
			PBF_AkinciSolverPtr->TOP_OPEN = getTopOpen();

			PBF_particles->x = &PBF_AkinciSolverPtr->Fluid->x;
			PBF_particles->v = &PBF_AkinciSolverPtr->Fluid->v;
			PBF_particles->a = &PBF_AkinciSolverPtr->Fluid->a;
			PBF_particles->m = &PBF_AkinciSolverPtr->Fluid->m;
			PBF_particles->V = &PBF_AkinciSolverPtr->Fluid->V;
			PBF_particles->rho = &PBF_AkinciSolverPtr->Fluid->rho;
			PBF_particles->nt = &PBF_AkinciSolverPtr->Fluid->neighbor_this;
			PBF_particles->no = &PBF_AkinciSolverPtr->Fluid->neighbor_others;

			PBF_particles->p_x = &PBF_AkinciSolverPtr->Fluid->p_x;
		}
			break;
		default:
			break;
	}

	inited = true;
}
void GAS_Hina_Solver_PBF::emit_data(SIM_Hina_Particles_PBF *PBF_particles)
{
	size_t max_p = getMaxNumOfParticles();
	bool one_shot = getIsOneShot();
	real spacing = getTargetSpacing();
	Vector start = getEmitStart();
	Vector end = getEmitEnd();

	HinaPE::FluidEmitter::UseFluidBlock(PBF_particles->x, start, end, spacing);

	emitted = one_shot;
}
