#ifndef HINAPE_SIM_HINA_PARTICLES_H
#define HINAPE_SIM_HINA_PARTICLES_H

#include <SIM_Hina_Generator.h>

namespace cuNSearch
{
class NeighborhoodSearch;
}
struct ParticleState
{
	GA_Offset pt_off;
	UT_Vector3 pt_pos;
};

SIM_HINA_GEOMETRY_CLASS(
		Particles,
		HINA_GETSET_PARAMETER(FluidDomain, GETSET_DATA_FUNCS_V3)
		HINA_GETSET_PARAMETER(TargetSpacing, GETSET_DATA_FUNCS_F)
		HINA_GETSET_PARAMETER(KernelRadiusOverTargetSpacing, GETSET_DATA_FUNCS_F)
		HINA_GETSET_PARAMETER(TargetDensity, GETSET_DATA_FUNCS_F)
		HINA_GETSET_PARAMETER(Kernel, GETSET_DATA_FUNCS_I)
		HINA_GETSET_PARAMETER(Gravity, GETSET_DATA_FUNCS_V3)

		fpreal UnivMass;
		std::map<GA_Offset, std::vector<ParticleState>> neighbor_lists_cache; // neighbors of this particles set TODO: remove this
		std::map<UT_String, std::map<GA_Offset, std::vector<ParticleState>>> other_neighbor_lists_cache; // neighbors of other particles sets TODO: remove this
		std::map<GA_Offset, GA_Size> offset2index;
		std::map<GA_Size, GA_Offset> index2offset;
		std::map<GA_Offset, UT_Vector3> position_cache;
		std::map<GA_Offset, UT_Vector3> velocity_cache;
		std::map<GA_Offset, UT_Vector3> force_cache;
		std::map<GA_Offset, fpreal> mass_cache;
		std::map<GA_Offset, fpreal> volume_cache;
		std::map<GA_Offset, fpreal> density_cache;
		virtual void load(); // load from gdp. generally, we never need to manually call this function
		virtual void commit(); // auto commit at `GAS_Hina_SubStep`
		virtual void calculate_mass(); // init phase, call once
		virtual void calculate_volume(); // after density is calculated, update every step
		void advect_position(fpreal dt);
		void advect_velocity(fpreal dt);
		void clear_force();
		void calculate_force_gravity();
		void force_keep_boundary();
		size_t size() const;
		void for_each_offset(const std::function<void(const GA_Offset &)> &func);
		void for_each_neighbor_self(const GA_Offset &pt_off, const std::function<void(const GA_Offset &, const UT_Vector3 &)> &func);
		void for_each_neighbor_others(const GA_Offset &pt_off, const std::function<void(const GA_Offset &, const UT_Vector3 &)> &func);
		void for_each_neighbor_others(const GA_Offset &pt_off, const std::function<void(const GA_Offset &, const UT_Vector3 &)> &func, const UT_String &other_name);

		UT_String obj_name;
		friend class GAS_Hina_BuildNeighborLists;
		GAS_Hina_BuildNeighborLists* neighbor_lists_builder;
)

#endif //HINAPE_SIM_HINA_PARTICLES_H
