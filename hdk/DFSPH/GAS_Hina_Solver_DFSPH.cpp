#include "GAS_Hina_Solver_DFSPH.h"

#include <memory>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		Solver_DFSPH,
		true,
		false,
		HINA_FLOAT_VECTOR_PARAMETER(FluidDomain, 3, 1., 1., 1.) \
        HINA_FLOAT_PARAMETER(TargetSpacing, .02) \
        HINA_FLOAT_PARAMETER(KernelRadius, .04) \
        HINA_FLOAT_PARAMETER(TargetDensity, 1000.) \
        HINA_FLOAT_VECTOR_PARAMETER(Gravity, 3, 0, -9.8, 0) \
		HINA_INT_PARAMETER(MaxNumOfParticles, 100000) \
        HINA_BOOL_PARAMETER(IsOneShot, true) \
        HINA_FLOAT_VECTOR_PARAMETER(EmitStart, 3, -.3, -.3, -.3) \
        HINA_FLOAT_VECTOR_PARAMETER(EmitEnd, 3, .3, .3, .3) \
        static std::array<PRM_Name, 4> Kernels = {\
            PRM_Name("0", "Poly64"), \
            PRM_Name("1", "Spiky"), \
            PRM_Name("2", "CubicSpline"), \
            PRM_Name(nullptr), \
		}; \
        static PRM_Name KernelName("Kernel", "Kernel"); \
        static PRM_Default KernelNameDefault(2, "CubicSpline"); \
        static PRM_ChoiceList CL(PRM_CHOICELIST_SINGLE, Kernels.data()); \
        PRMS.emplace_back(PRM_ORD, 1, &KernelName, &KernelNameDefault, &CL); \
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
		init_data(DFSPH_particles);

	if (!emitted)
		emit_data(DFSPH_particles);

	SolverPtr->Solve(timestep);

	return true;
}
void GAS_Hina_Solver_DFSPH::init_data(SIM_Hina_Particles_DFSPH *DFSPH_particles)
{
	real kernel_radius = getKernelRadius();
	Vector domain = getFluidDomain();
	real spacing = getTargetSpacing();
	real rest_density = getTargetDensity();
	Vector gravity = getGravity();

	SolverPtr = std::make_shared<HinaPE::DFSPH1Solver>(kernel_radius, domain);
	SolverPtr->FLUID_REST_DENSITY = rest_density;
	SolverPtr->FLUID_PARTICLE_RADIUS = spacing / 2.;
	SolverPtr->GRAVITY = gravity;

	DFSPH_particles->x = &SolverPtr->Fluid.x;
	DFSPH_particles->v = &SolverPtr->Fluid.v;
	DFSPH_particles->a = &SolverPtr->Fluid.a;
	DFSPH_particles->m = &SolverPtr->Fluid.m;
	DFSPH_particles->V = &SolverPtr->Fluid.V;
	DFSPH_particles->rho = &SolverPtr->Fluid.rho;
	DFSPH_particles->nt = &SolverPtr->Fluid.neighbor_this;
	DFSPH_particles->no = &SolverPtr->Fluid.neighbor_others;
	DFSPH_particles->factor = &SolverPtr->Fluid.factor;
	DFSPH_particles->density_adv = &SolverPtr->Fluid.density_adv;

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
