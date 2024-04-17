#include "GAS_Hina_Solver_PBF.h"
#include <Base/utils.h>

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
        HINA_BOOL_PARAMETER(UseFluidBlock, false) \
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
        UpdateAllAkinciBoundaries(obj);
		PBF_AkinciSolverPtr->Solve(timestep);
        apply_akinci_force(obj);
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
            //PBF_AkinciSolverPtr = std::make_shared<HinaPE::PbfSolver>(static_cast<real>(getKernelRadius()), getFluidDomainF());
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

            PBF_particles->pred_x = &PBF_AkinciSolverPtr->Fluid->pred_x;
            PBF_particles->lambda = &PBF_AkinciSolverPtr->Fluid->lambda;
            PBF_particles->delta_p = &PBF_AkinciSolverPtr->Fluid->delta_p;
            PBF_particles->a_ext = &PBF_AkinciSolverPtr->Fluid->a_ext;


            std::vector<SIM_Hina_Particles_Akinci *> akinci_boundaries = FetchAllAkinciBoundaries(obj);
            for (auto &akinci_boundary: akinci_boundaries)
            {
                PBF_AkinciSolverPtr->Boundaries.emplace_back(std::make_shared<HinaPE::AkinciBoundaryPBF>());
                akinci_boundary->x = &PBF_AkinciSolverPtr->Boundaries.back()->x;
                akinci_boundary->v = &PBF_AkinciSolverPtr->Boundaries.back()->v;
                akinci_boundary->a = &PBF_AkinciSolverPtr->Boundaries.back()->a;
                akinci_boundary->m = &PBF_AkinciSolverPtr->Boundaries.back()->m;
                akinci_boundary->V = &PBF_AkinciSolverPtr->Boundaries.back()->V;
                akinci_boundary->rho = &PBF_AkinciSolverPtr->Boundaries.back()->rho;
                akinci_boundary->nt = &PBF_AkinciSolverPtr->Boundaries.back()->neighbor_this;
                akinci_boundary->no = &PBF_AkinciSolverPtr->Boundaries.back()->neighbor_others;
                akinci_boundary->x_init = &PBF_AkinciSolverPtr->Boundaries.back()->x_init;
                akinci_boundary->xform = &PBF_AkinciSolverPtr->Boundaries.back()->xform;
                akinci_boundary->b_set_index = PBF_AkinciSolverPtr->Boundaries.size() - 1;

                akinci_boundary->SP = &PBF_AkinciSolverPtr->Boundaries.back()->boundary_sp;
                akinci_boundary->normals = &PBF_AkinciSolverPtr->Boundaries.back()->normals;
                akinci_boundary->u_diff = &PBF_AkinciSolverPtr->Boundaries.back()->u_diff;

                PBF_AkinciSolverPtr->BOUNDARY_REST_DENSITY.emplace_back(static_cast<real>(akinci_boundary->getSolidDensity()));
                PBF_AkinciSolverPtr->BOUNDARY_DYNAMICS.emplace_back(akinci_boundary->getIsDynamic());
            }

            std::vector<SIM_Hina_SDF_Boundary *> sdf_boundaries = FetchAllSDFBoundaries(obj);
            for (auto &sdf_boundary: sdf_boundaries)
            {
                PBF_AkinciSolverPtr->SDFBoundaries.emplace_back(std::make_shared<HinaPE::SDFBoundaryPBF>());
                PBF_AkinciSolverPtr->SDFBoundaries.back()->S = sdf_boundary->S; // Notice here, S is first load into SDFBoundaries, then SDFBoundaries is loaded into SDF_AkinciSolver

                PBF_AkinciSolverPtr->SDF_FRICTION.emplace_back(sdf_boundary->getFriction());
                PBF_AkinciSolverPtr->SDF_BOUNCINESS.emplace_back(sdf_boundary->getBounciness());
                PBF_AkinciSolverPtr->SDF_DYNAMICS.emplace_back(sdf_boundary->getIsDynamic());
            }
		}
			break;
		default:
			break;
	}

    InitAllAkinciBoundaries(obj);
	inited = true;
}
void GAS_Hina_Solver_PBF::emit_data(SIM_Hina_Particles_PBF *PBF_particles)
{
    size_t max_p = getMaxNumOfParticles();
    real spacing = getTargetSpacing();
    Vector start = getEmitStart();
    Vector end = getEmitEnd();

    SIM_Geometry *sub_geometry_as_emitter_source = SIM_DATA_GET(*this, SIM_GEOMETRY_DATANAME, SIM_Geometry);
    if (sub_geometry_as_emitter_source)
    {
        Vector pos;
        std::pair<std::vector<Vector>, std::vector<size_t>> triangle_mesh_info = ReadTriangleMeshFromGeometry<real, Vector>(sub_geometry_as_emitter_source, pos);
        VectorArrayCPU temp;
        HinaPE::FluidEmitter::UseTriangleMeshSource(&temp, triangle_mesh_info, pos, 1.6 * spacing, max_p);
        PBF_particles->x->insert(PBF_particles->x->end(), temp.begin(), temp.end());
    }

    if (getUseFluidBlock())
    {
        VectorArrayCPU temp;
        HinaPE::FluidEmitter::UseFluidBlock(&temp, start, end, spacing);
        PBF_particles->x->insert(PBF_particles->x->end(), temp.begin(), temp.end());
    }
    emitted = true;
}

void GAS_Hina_Solver_PBF::apply_akinci_force(SIM_Object *obj)
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

                    rigidbody->rb->setAngularDamping(boundary->getRotationStiffness());
                    //rigidbody->rb->resetTorque();
                }
            }
        }
    }
}