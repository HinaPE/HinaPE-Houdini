#ifndef HINAPE_SIM_HINA_PARTICLES_H
#define HINAPE_SIM_HINA_PARTICLES_H

#include <SIM_Hina_Generator.h>
#include <HinaPE/DFSPH/DFSPH.h>

SIM_HINA_GEOMETRY_CLASS(
		Particles,
		HINA_GETSET_PARAMETER(FluidDomain, GETSET_DATA_FUNCS_V3)
		HINA_GETSET_PARAMETER(TargetSpacing, GETSET_DATA_FUNCS_F)
		HINA_GETSET_PARAMETER(KernelRadiusOverTargetSpacing, GETSET_DATA_FUNCS_F)
		HINA_GETSET_PARAMETER(TargetDensity, GETSET_DATA_FUNCS_F)
		HINA_GETSET_PARAMETER(Kernel, GETSET_DATA_FUNCS_I)
		HINA_GETSET_PARAMETER(Gravity, GETSET_DATA_FUNCS_V3)

		virtual void load();
		virtual void commit(); // auto commit by `GAS_Hina_SubStep`
		std::map<GA_Offset, GA_Size> offset2index;
		std::map<GA_Size, GA_Offset> index2offset;
		HinaPE::CPUVectorArray *x, *v, *f;
		HinaPE::CPUScalarArray *m, *V, *rho;
)

#endif //HINAPE_SIM_HINA_PARTICLES_H
