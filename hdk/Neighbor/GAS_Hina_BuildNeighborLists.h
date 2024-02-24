#ifndef HINAPE_GAS_HINA_BUILDNEIGHBORLISTS_H
#define HINAPE_GAS_HINA_BUILDNEIGHBORLISTS_H

#include <SIM_Hina_Generator.h>

namespace cuNSearch
{
	class NeighborhoodSearch;
}
class SIM_Hina_Particles;

GAS_HINA_SUBSOLVER_CLASS(
		BuildNeighborLists,
		HINA_GETSET_PARAMETER(KernelRadius, GETSET_DATA_FUNCS_F)
		HINA_GETSET_PARAMETER(Backend, GETSET_DATA_FUNCS_I)

		void init_search_engine(SIM_Object *fluid_obj);
		void update_search_engine(SIM_Object *fluid_obj);

private:
		void _add_particle_set(const UT_String& name, SIM_Hina_Particles* particles);
		void _update_particle_set(const UT_String& name, SIM_Hina_Particles* particles);
		void _update_neighbor(const UT_String& name, SIM_Hina_Particles* particles);
		cuNSearch::NeighborhoodSearch* nsearch;
		std::map<UT_String, std::vector<std::array<fpreal, 3>>> cached_positions;
		std::map<UT_String, unsigned int> cached_point_set_indices;
)

#endif //HINAPE_HOUDINI_GAS_HINA_BUILDNEIGHBORLISTS_H
