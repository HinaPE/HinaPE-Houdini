#ifndef HINAPE_GAS_HINA_GRIDSOURCEEMITTER_H
#define HINAPE_GAS_HINA_GRIDSOURCEEMITTER_H

#include <SIM_Hina_Generator.h>

GAS_HINA_SUBSOLVER_CLASS(
		GridSourceEmitter,
		HINA_GETSET_PARAMETER(EmitOnce, GETSET_DATA_FUNCS_B)

		THREADED_METHOD2(GAS_Hina_GridSourceEmitter, OutField->shouldMultiThread(), _emit, SIM_RawField * , OutField, const SIM_RawField * , InField);
		void _emitPartial(SIM_RawField *OutField, const SIM_RawField *InField, const UT_JobInfo &info);
		bool emitted;
)

#endif //HINAPE_GAS_HINA_GRIDSOURCEEMITTER_H
