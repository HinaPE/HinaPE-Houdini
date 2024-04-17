#ifndef HINAPE_GAS_HINA_SOLVER_PBF_H
#define HINAPE_GAS_HINA_SOLVER_PBF_H

#include <SIM_Hina_Generator.h>

#include <PBF/SIM_Hina_Particles_PBF.h>
#include <HinaPE/PBF_Akinci/PBF_Akinci.h>
//#include <HinaPE/PBF/PBF.h>
#include <Boundary/SIM_Hina_SDF_Boundary.h>
#include <Rigid/SIM_Hina_RigidBody.h>
#include <Boundary/SIM_Hina_Particles_Akinci.h>

GAS_HINA_SUBSOLVER_CLASS(
		Solver_PBF,
        HINA_GETSET_PARAMETER(FluidDomain, GETSET_DATA_FUNCS_V3)
        HINA_GETSET_PARAMETER(TopOpen, GETSET_DATA_FUNCS_B)
        HINA_GETSET_PARAMETER(TargetSpacing, GETSET_DATA_FUNCS_F)
        HINA_GETSET_PARAMETER(KernelRadius, GETSET_DATA_FUNCS_F)
        HINA_GETSET_PARAMETER(TargetDensity, GETSET_DATA_FUNCS_F)
        HINA_GETSET_PARAMETER(FluidViscosity, GETSET_DATA_FUNCS_F)
        HINA_GETSET_PARAMETER(FluidSurfaceTension, GETSET_DATA_FUNCS_F)
        HINA_GETSET_PARAMETER(Gravity, GETSET_DATA_FUNCS_V3)
        HINA_GETSET_PARAMETER(MaxNumOfParticles, GETSET_DATA_FUNCS_I)
        HINA_GETSET_PARAMETER(UseFluidBlock, GETSET_DATA_FUNCS_B)
        HINA_GETSET_PARAMETER(EmitStart, GETSET_DATA_FUNCS_V3)
        HINA_GETSET_PARAMETER(EmitEnd, GETSET_DATA_FUNCS_V3)
        HINA_GETSET_PARAMETER(Kernel, GETSET_DATA_FUNCS_I)
        HINA_GETSET_PARAMETER(BoundaryHandling, GETSET_DATA_FUNCS_I)

		std::shared_ptr<HinaPE::PBF_AkinciSolver> PBF_AkinciSolverPtr;
        //std::shared_ptr<HinaPE::PbfSolver> PBF_AkinciSolverPtr;
		bool inited;
		bool emitted;

		void init_data(SIM_Hina_Particles_PBF *, SIM_Object *);
		void emit_data(SIM_Hina_Particles_PBF *);
        void apply_akinci_force(SIM_Object *);
)

#endif //HINAPE_GAS_HINA_SOLVER_PBF_H
