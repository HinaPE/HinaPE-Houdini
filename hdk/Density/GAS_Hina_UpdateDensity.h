#ifndef HINAPE_HOUDINI_GAS_HINA_UPDATEDENSITY_H
#define HINAPE_HOUDINI_GAS_HINA_UPDATEDENSITY_H

#include <SIM_Hina_Generator.h>

class SIM_Hina_Particles;

GAS_HINA_SUBSOLVER_CLASS(
		UpdateDensity,
		HINA_GETSET_PARAMETER(Kernel, GETSET_DATA_FUNCS_I)

		static void calculate_density(SIM_Hina_Particles *particles);
)

#endif //HINAPE_HOUDINI_GAS_HINA_UPDATEDENSITY_H
