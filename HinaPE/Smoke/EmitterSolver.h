#ifndef HINAPE_EMITTERSOLVER_H
#define HINAPE_EMITTERSOLVER_H

#include <SIM/SIM_RawField.h>

namespace HinaPE
{
struct EmitterSolver
{
	void solve(SIM_RawField *IO_Field, SIM_RawField *IO_V_X, SIM_RawField *IO_V_Y, SIM_RawField *IO_V_Z);

private:
	THREADED_METHOD4(EmitterSolver, IO_Field->shouldMultiThread(), _emit, SIM_RawField * , IO_Field, SIM_RawField *, IO_V_X, SIM_RawField *, IO_V_Y, SIM_RawField *, IO_V_Z);
	void _emitPartial(SIM_RawField *IO_Field, SIM_RawField *IO_V_X, SIM_RawField *IO_V_Y, SIM_RawField *IO_V_Z, const UT_JobInfo &info);
};
}

#endif //HINAPE_EMITTERSOLVER_H
