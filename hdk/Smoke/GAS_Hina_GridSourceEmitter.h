#ifndef HINAPE_HOUDINI_GAS_HINA_GRIDSOURCEEMITTER_H
#define HINAPE_HOUDINI_GAS_HINA_GRIDSOURCEEMITTER_H

#include <SIM_Hina_Generator.h>

GAS_HINA_SUBSOLVER_CLASS(
		GridSourceEmitter,
		HINA_GETSET_PARAMETER(EmitOnce, GETSET_DATA_FUNCS_B)

		THREADED_METHOD2(SmokeNativeSolver, Src->shouldMultiThread(), _emit, SIM_RawField * , Dest, const SIM_RawField * , Src);
		void _emitPartial(SIM_RawField *Dest, const SIM_RawField *Src, const UT_JobInfo &info);
		bool emitted;
)

#endif //HINAPE_HOUDINI_GAS_HINA_GRIDSOURCEEMITTER_H
