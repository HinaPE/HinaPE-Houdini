#ifndef HINAPE_GAS_HINA_GRIDEXTERNALFORCE_H
#define HINAPE_GAS_HINA_GRIDEXTERNALFORCE_H

#include <SIM_Hina_Generator.h>

GAS_HINA_SUBSOLVER_CLASS(
		GridExternalForce,
		HINA_GETSET_PARAMETER(Gravity, GETSET_DATA_FUNCS_F)

		THREADED_METHOD4(SmokeNativeSolver, V_X->shouldMultiThread(), _apply, float, dt, SIM_RawField * , V_X, SIM_RawField * , V_Y, SIM_RawField * , V_Z);
		void _applyPartial(float dt, SIM_RawField *V_X, SIM_RawField *V_Y, SIM_RawField *V_Z, const UT_JobInfo &info);
)

#endif //HINAPE_GAS_HINA_GRIDEXTERNALFORCE_H
