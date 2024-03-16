#ifndef HINAPE_GAS_HINA_SOLVER_DFSPH_H
#define HINAPE_GAS_HINA_SOLVER_DFSPH_H

#include <SIM_Hina_Generator.h>

#include <DFSPH/SIM_Hina_Particles_DFSPH.h>
#include <Boundary/SIM_Hina_Particles_Akinci.h>
#include <Boundary/SIM_Hina_SDF_Boundary.h>
#include <Rigid/SIM_Hina_RigidBody.h>
#include <HinaPE/DFSPH_Akinci/DFSPH_Akinci.h>

GAS_HINA_SUBSOLVER_CLASS(
		Solver_DFSPH,
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

		std::shared_ptr<HinaPE::DFSPH_AkinciSolver> DFSPH_AkinciSolverPtr;
		bool inited;
		bool emitted;

		void init_data(SIM_Hina_Particles_DFSPH*, SIM_Object *);
		void emit_data(SIM_Hina_Particles_DFSPH*);
		void apply_akinci_force(SIM_Object *);
)

#endif //HINAPE_GAS_HINA_SOLVER_DFSPH_H
