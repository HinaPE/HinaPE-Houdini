#include "GAS_Hina_Solver_DFSPH.h"
#include <Akinci2012/SIM_Hina_Particles_Akinci.h>
#include <memory>

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
GAS_Hina_Solver_DFSPH::~GAS_Hina_Solver_DFSPH()
{
	this->SolverPtr = nullptr;
	this->inited = false;
	this->emitted = false;
}
void GAS_Hina_Solver_DFSPH::_init()
{
	this->SolverPtr = nullptr;
	this->inited = false;
	this->emitted = false;
}
void GAS_Hina_Solver_DFSPH::_makeEqual(const GAS_Hina_Solver_DFSPH *src)
{
	this->SolverPtr = src->SolverPtr;
	this->inited = src->inited;
	this->emitted = src->emitted;
}
bool GAS_Hina_Solver_DFSPH::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_Hina_Particles_DFSPH *DFSPH_particles = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_Particles_DFSPH);
	CHECK_NULL_RETURN_BOOL(DFSPH_particles)

	if (!inited)
		init_data(DFSPH_particles, obj);

	if (!emitted)
		emit_data(DFSPH_particles);

	SolverPtr->Solve(timestep);

	return true;
}
void GAS_Hina_Solver_DFSPH::init_data(SIM_Hina_Particles_DFSPH *DFSPH_particles, SIM_Object *obj)
{
	real kernel_radius = getKernelRadius();
	Vector domain = getFluidDomain();
	real spacing = getTargetSpacing();
	real rest_density = getTargetDensity();
	Vector gravity = getGravity();
	bool top_open = getTopOpen();

	SolverPtr = std::make_shared<HinaPE::DFSPH_AkinciSolver>(kernel_radius, domain);
	SolverPtr->FLUID_REST_DENSITY = rest_density;
	SolverPtr->FLUID_PARTICLE_RADIUS = spacing / 2.;
	SolverPtr->GRAVITY = gravity;
	SolverPtr->TOP_OPEN = top_open;

	DFSPH_particles->x = &SolverPtr->Fluid->x;
	DFSPH_particles->v = &SolverPtr->Fluid->v;
	DFSPH_particles->a = &SolverPtr->Fluid->a;
	DFSPH_particles->m = &SolverPtr->Fluid->m;
	DFSPH_particles->V = &SolverPtr->Fluid->V;
	DFSPH_particles->rho = &SolverPtr->Fluid->rho;
	DFSPH_particles->nt = &SolverPtr->Fluid->neighbor_this;
	DFSPH_particles->no = &SolverPtr->Fluid->neighbor_others;
	DFSPH_particles->factor = &SolverPtr->Fluid->factor;
	DFSPH_particles->k = &SolverPtr->Fluid->k;
	DFSPH_particles->density_adv = &SolverPtr->Fluid->density_adv;

	int boundary_handling = getBoundaryHandling();
	switch (boundary_handling)
	{
		case 0: // Akinci2012
		{
			std::vector<SIM_Hina_Particles_Akinci *> akinci_boundaries = FetchAllAkinciBoundaries(obj);
			for (auto &akinci_boundary: akinci_boundaries)
			{
				SolverPtr->Boundaries.emplace_back(std::make_shared<HinaPE::AkinciBoundaryCPU>());
				akinci_boundary->x = &SolverPtr->Boundaries.back()->x;
				akinci_boundary->v = &SolverPtr->Boundaries.back()->v;
				akinci_boundary->a = &SolverPtr->Boundaries.back()->a;
				akinci_boundary->m = &SolverPtr->Boundaries.back()->m;
				akinci_boundary->V = &SolverPtr->Boundaries.back()->V;
				akinci_boundary->rho = &SolverPtr->Boundaries.back()->rho;
				akinci_boundary->nt = &SolverPtr->Boundaries.back()->neighbor_this;
				akinci_boundary->no = &SolverPtr->Boundaries.back()->neighbor_others;

				akinci_boundary->load(); // load from gdp to HinaPE
				SolverPtr->Boundaries.back()->size = akinci_boundary->x->size();
				SolverPtr->BOUNDARY_REST_DENSITY.emplace_back(static_cast<real>(akinci_boundary->getSolidDensity()));
			}
		}
			break;
		case 1: // Koschier2017
		{
		}
			break;
		case 2: // Bender2019
		{
		}
			break;
		case 3: // None
		{
		}
			break;
		default:
			break;
	}

	inited = true;
}
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
