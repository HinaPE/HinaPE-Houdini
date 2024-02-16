#ifndef HINAPE_HOUDINI_GAS_HINA_UPDATEDENSITY_H
#define HINAPE_HOUDINI_GAS_HINA_UPDATEDENSITY_H

#include <SIM_Hina_Generator.h>

GAS_HINA_SUBSOLVER_CLASS(
		UpdateDensity,
		HINA_GETSET_PARAMETER(KernelRadius, GETSET_DATA_FUNCS_F)
		HINA_GETSET_PARAMETER(Kernel, GETSET_DATA_FUNCS_I)
)

#endif //HINAPE_HOUDINI_GAS_HINA_UPDATEDENSITY_H
