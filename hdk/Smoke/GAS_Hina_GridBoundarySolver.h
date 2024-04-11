#ifndef HINAPE_GAS_HINA_GRIDBOUNDARYSOLVER_H
#define HINAPE_GAS_HINA_GRIDBOUNDARYSOLVER_H

#include <SIM_Hina_Generator.h>

GAS_HINA_SUBSOLVER_CLASS(
		GridBoundarySolver,
		HINA_GETSET_PARAMETER(Iter, GETSET_DATA_FUNCS_I)

		THREADED_METHOD2(GAS_Hina_GridBoundarySolver, Field->shouldMultiThread(), _extrapolate, SIM_RawField *, Field, SIM_RawField *, Valid);
		void _extrapolatePartial(SIM_RawField *Field, SIM_RawField *Valid, const UT_JobInfo &info);
)

#endif //HINAPE_GAS_HINA_GRIDBOUNDARYSOLVER_H
