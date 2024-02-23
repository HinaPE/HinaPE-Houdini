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

		fpreal Mass;
		std::map<GA_Offset, std::vector<std::pair<GA_Offset, UT_Vector3>>> neighbor_lists_cache; // neighbors of this particles set
		std::map<UT_String, std::map<GA_Offset, std::vector<std::pair<GA_Offset, UT_Vector3>>>> other_neighbor_lists_cache; // neighbors of other particles sets
		std::map<GA_Offset, UT_Vector3> positions_cache;
		std::map<GA_Offset, UT_Vector3> velocity_cache;
		virtual void commit(); // Commit Caches to GDP
		virtual void calculate_mass();
		virtual void calculate_volume();
		void calculate_density(); // call after calculate neighbors
		void for_each_neighbor_self(const GA_Offset &pt_off, std::function<void(const GA_Offset &, const UT_Vector3 &)> func);
		void for_each_neighbor_others(const GA_Offset &pt_off, std::function<void(const GA_Offset &, const UT_Vector3 &)> func);
		void for_each_neighbor_others(const GA_Offset &pt_off, std::function<void(const GA_Offset &, const UT_Vector3 &)> func, const UT_String &other_name);
)

GAS_HINA_SUBSOLVER_CLASS(
		CommitCache,
)

#endif //HINAPE_SIM_HINA_PARTICLES_H
