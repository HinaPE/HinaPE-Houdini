#ifndef HINAPE_SIM_HINA_PARTICLES_H
#define HINAPE_SIM_HINA_PARTICLES_H

#include <SIM_Hina_Generator.h>

SIM_HINA_GEOMETRY_CLASS(
		Particles,
		HINA_GETSET_PARAMETER(FluidDomain, GETSET_DATA_FUNCS_V3)
		HINA_GETSET_PARAMETER(TargetSpacing, GETSET_DATA_FUNCS_F)
		HINA_GETSET_PARAMETER(KernelRadiusOverTargetSpacing, GETSET_DATA_FUNCS_F)
		HINA_GETSET_PARAMETER(TargetDensity, GETSET_DATA_FUNCS_F)
		HINA_GETSET_PARAMETER(Kernel, GETSET_DATA_FUNCS_I)

		virtual void Commit(); // Commit Caches to GDP
		mutable fpreal Mass;
		std::map<GA_Offset, std::vector<GA_Offset>> neighbor_lists_cache; // neighbors of this particles set
		std::map<UT_String, std::map<GA_Offset, std::vector<GA_Offset>>> other_neighbor_lists_cache; // neighbors of other particles sets
		std::map<GA_Offset, UT_Vector3> positions_cache;
		std::map<GA_Offset, UT_Vector3> velocity_cache;
)

GAS_HINA_SUBSOLVER_CLASS(
		CommitCache,
)

#endif //HINAPE_SIM_HINA_PARTICLES_H
