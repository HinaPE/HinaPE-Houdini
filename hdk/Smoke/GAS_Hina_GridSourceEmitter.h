#ifndef HINAPE_GAS_HINA_GRIDSOURCEEMITTER_H
#define HINAPE_GAS_HINA_GRIDSOURCEEMITTER_H

#include <SIM_Hina_Generator.h>

GAS_HINA_SUBSOLVER_CLASS(
		GridSourceEmitter,
		HINA_GETSET_PARAMETER(EmitOnce, GETSET_DATA_FUNCS_B)
		bool emitted;
)

#endif //HINAPE_GAS_HINA_GRIDSOURCEEMITTER_H
