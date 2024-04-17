#ifndef HINAPE_EMITTERSOLVER_H
#define HINAPE_EMITTERSOLVER_H

#include <SIM/SIM_RawField.h>

namespace HinaPE
{
struct EmitterSolver
{
	THREADED_METHOD5(GAS_Hina_GridSourceEmitter, OutField->shouldMultiThread(), _emit, SIM_RawField * , OutField, const SIM_RawField *, InField, SIM_RawField *, OutFlow_X, SIM_RawField *, OutFlow_Y, SIM_RawField *, OutFlow_Z);
	void _emitPartial(SIM_RawField *OutField, const SIM_RawField *InField, SIM_RawField *OutFlow_X, SIM_RawField *OutFlow_Y, SIM_RawField *OutFlow_Z, const UT_JobInfo &info);
};
}

#endif //HINAPE_EMITTERSOLVER_H
