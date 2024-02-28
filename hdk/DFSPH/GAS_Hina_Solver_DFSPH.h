#ifndef HINAPE_GAS_HINA_SOLVER_DFSPH_H
#define HINAPE_GAS_HINA_SOLVER_DFSPH_H

#include <SIM_Hina_Generator.h>
#include <DFSPH/SIM_Hina_Particles_DFSPH.h>
#include <HinaPE/DFSPH1/DFSPH1.h>

GAS_HINA_SUBSOLVER_CLASS(
		Solver_DFSPH,
		HINA_GETSET_PARAMETER(FluidDomain, GETSET_DATA_FUNCS_V3)
		HINA_GETSET_PARAMETER(TargetSpacing, GETSET_DATA_FUNCS_F)
		HINA_GETSET_PARAMETER(KernelRadius, GETSET_DATA_FUNCS_F)
		HINA_GETSET_PARAMETER(TargetDensity, GETSET_DATA_FUNCS_F)
		HINA_GETSET_PARAMETER(Kernel, GETSET_DATA_FUNCS_I)
		HINA_GETSET_PARAMETER(Gravity, GETSET_DATA_FUNCS_V3)
		HINA_GETSET_PARAMETER(MaxNumOfParticles, GETSET_DATA_FUNCS_I)
		HINA_GETSET_PARAMETER(IsOneShot, GETSET_DATA_FUNCS_B)
		HINA_GETSET_PARAMETER(EmitStart, GETSET_DATA_FUNCS_V3)
		HINA_GETSET_PARAMETER(EmitEnd, GETSET_DATA_FUNCS_V3)

		std::shared_ptr<HinaPE::DFSPH1Solver> SolverPtr;
		bool inited;
		bool emitted;

		void init_data(SIM_Hina_Particles_DFSPH*);
		void emit_data(SIM_Hina_Particles_DFSPH*);
)

#endif //HINAPE_GAS_HINA_SOLVER_DFSPH_H
