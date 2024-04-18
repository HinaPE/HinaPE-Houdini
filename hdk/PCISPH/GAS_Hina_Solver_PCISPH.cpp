//
// Created by LiYifan on 2024/4/18.
//

#include "GAS_Hina_Solver_PCISPH.h"
#include <Base/utils.h>

GAS_HINA_SUBSOLVER_IMPLEMENT(
        Solver_PCISPH,
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
        TARGET_PARTICLE_GEOMETRY(SIM_Hina_Particles_PCISPH)
)

void GAS_Hina_Solver_PCISPH::_init()
{
    this->PCISPH_AkinciSolverPtr = nullptr;
    this->inited = false;
    this->emitted = false;
}

void GAS_Hina_Solver_PCISPH::_makeEqual(const GAS_Hina_Solver_PCISPH *src)
{
    this->PCISPH_AkinciSolverPtr = src->PCISPH_AkinciSolverPtr;
    this->inited = src->inited;
    this->emitted = src->emitted;
}

bool GAS_Hina_Solver_PCISPH::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
    SIM_Hina_Particles_PCISPH *PCISPH_particles = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_Particles_PCISPH);
    if (PCISPH_particles == nullptr)
        return true;

    if (!inited)
        init_data(PCISPH_particles, obj);

    if (!emitted)
        emit_data(PCISPH_particles);

    if (PCISPH_AkinciSolverPtr)
    {
        UpdateAllAkinciBoundaries(obj);
        PCISPH_AkinciSolverPtr->Solve(timestep);
        apply_akinci_force(obj);
    }
    return true;
}

void GAS_Hina_Solver_PCISPH::init_data(SIM_Hina_Particles_PCISPH *PCISPH_particles, SIM_Object *obj)
{
    switch (getBoundaryHandling())
    {
        case 0: // Akinci2012
        {
            PCISPH_AkinciSolverPtr = std::make_shared<HinaPE::PCISPH_AkinciSolver>(static_cast<real>(getKernelRadius()), getFluidDomainF());
            //PCISPH_AkinciSolverPtr = std::make_shared<HinaPE::PbfSolver>(static_cast<real>(getKernelRadius()), getFluidDomainF());
            PCISPH_AkinciSolverPtr->FLUID_REST_DENSITY = getTargetDensity();
            PCISPH_AkinciSolverPtr->FLUID_PARTICLE_RADIUS = getTargetSpacing() / 2.;
            PCISPH_AkinciSolverPtr->GRAVITY = getGravityF();
            PCISPH_AkinciSolverPtr->TOP_OPEN = getTopOpen();

            PCISPH_particles->x = &PCISPH_AkinciSolverPtr->Fluid->x;
            PCISPH_particles->v = &PCISPH_AkinciSolverPtr->Fluid->v;
            PCISPH_particles->a = &PCISPH_AkinciSolverPtr->Fluid->a;
            PCISPH_particles->m = &PCISPH_AkinciSolverPtr->Fluid->m;
            PCISPH_particles->V = &PCISPH_AkinciSolverPtr->Fluid->V;
            PCISPH_particles->rho = &PCISPH_AkinciSolverPtr->Fluid->rho;
            PCISPH_particles->nt = &PCISPH_AkinciSolverPtr->Fluid->neighbor_this;
            PCISPH_particles->no = &PCISPH_AkinciSolverPtr->Fluid->neighbor_others;

            PCISPH_particles->pred_x = &PCISPH_AkinciSolverPtr->Fluid->pred_x;
            PCISPH_particles->pred_v = &PCISPH_AkinciSolverPtr->Fluid->pred_v;
            PCISPH_particles->pred_density = &PCISPH_AkinciSolverPtr->Fluid->pred_density;
            PCISPH_particles->pressure = &PCISPH_AkinciSolverPtr->Fluid->pressure;
            PCISPH_particles->d_error = &PCISPH_AkinciSolverPtr->Fluid->d_error;
            PCISPH_particles->a_ext = &PCISPH_AkinciSolverPtr->Fluid->a_ext;
            PCISPH_particles->a_pressure = &PCISPH_AkinciSolverPtr->Fluid->a_pressure;
            PCISPH_particles->delta = &PCISPH_AkinciSolverPtr->Fluid->delta;

            std::vector<SIM_Hina_Particles_Akinci *> akinci_boundaries = FetchAllAkinciBoundaries(obj);
            for (auto &akinci_boundary: akinci_boundaries)
            {
                PCISPH_AkinciSolverPtr->Boundaries.emplace_back(std::make_shared<HinaPE::AkinciBoundaryPCISPH>());
                akinci_boundary->x = &PCISPH_AkinciSolverPtr->Boundaries.back()->x;
                akinci_boundary->v = &PCISPH_AkinciSolverPtr->Boundaries.back()->v;
                akinci_boundary->a = &PCISPH_AkinciSolverPtr->Boundaries.back()->a;
                akinci_boundary->m = &PCISPH_AkinciSolverPtr->Boundaries.back()->m;
                akinci_boundary->V = &PCISPH_AkinciSolverPtr->Boundaries.back()->V;
                akinci_boundary->rho = &PCISPH_AkinciSolverPtr->Boundaries.back()->rho;
                akinci_boundary->nt = &PCISPH_AkinciSolverPtr->Boundaries.back()->neighbor_this;
                akinci_boundary->no = &PCISPH_AkinciSolverPtr->Boundaries.back()->neighbor_others;
                akinci_boundary->x_init = &PCISPH_AkinciSolverPtr->Boundaries.back()->x_init;
                akinci_boundary->xform = &PCISPH_AkinciSolverPtr->Boundaries.back()->xform;
                akinci_boundary->b_set_index = PCISPH_AkinciSolverPtr->Boundaries.size() - 1;

                akinci_boundary->SP = &PCISPH_AkinciSolverPtr->Boundaries.back()->boundary_sp;
                akinci_boundary->normals = &PCISPH_AkinciSolverPtr->Boundaries.back()->normals;
                akinci_boundary->u_diff = &PCISPH_AkinciSolverPtr->Boundaries.back()->u_diff;

                PCISPH_AkinciSolverPtr->BOUNDARY_REST_DENSITY.emplace_back(static_cast<real>(akinci_boundary->getSolidDensity()));
                PCISPH_AkinciSolverPtr->BOUNDARY_DYNAMICS.emplace_back(akinci_boundary->getIsDynamic());
            }
        }
            break;
        default:
            break;
    }

    InitAllAkinciBoundaries(obj);
    inited = true;
}

void GAS_Hina_Solver_PCISPH::emit_data(SIM_Hina_Particles_PCISPH *PCISPH_particles)
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
        PCISPH_particles->x->insert(PCISPH_particles->x->end(), temp.begin(), temp.end());
    }

    if (getUseFluidBlock())
    {
        VectorArrayCPU temp;
        HinaPE::FluidEmitter::UseFluidBlock(&temp, start, end, spacing);
        PCISPH_particles->x->insert(PCISPH_particles->x->end(), temp.begin(), temp.end());
    }
    emitted = true;
}

void GAS_Hina_Solver_PCISPH::apply_akinci_force(SIM_Object *obj)
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