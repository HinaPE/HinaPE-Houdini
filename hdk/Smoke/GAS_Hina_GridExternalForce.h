#ifndef HINAPE_GAS_HINA_GRIDEXTERNALFORCE_H
#define HINAPE_GAS_HINA_GRIDEXTERNALFORCE_H

#include <SIM_Hina_Generator.h>

GAS_HINA_SUBSOLVER_CLASS(
		GridExternalForce,
		HINA_GETSET_PARAMETER(Gravity, GETSET_DATA_FUNCS_F)
		HINA_GETSET_PARAMETER(DensityFactor, GETSET_DATA_FUNCS_F)
		HINA_GETSET_PARAMETER(TemperatureFactor, GETSET_DATA_FUNCS_F)

		THREADED_METHOD2(GAS_Hina_GridExternalForce, V_Y->shouldMultiThread(), _apply_gravity, float, dt, SIM_RawField * , V_Y);
		void _apply_gravityPartial(float dt, SIM_RawField *V_Y, const UT_JobInfo &info);

		THREADED_METHOD5(GAS_Hina_GridExternalForce, V_Y->shouldMultiThread(), _apply_buoyancy, float, dt, SIM_RawField * , V_Y, const SIM_RawField * , D, const SIM_RawField * , T, fpreal, t_amb);
		void _apply_buoyancyPartial(float dt, SIM_RawField *V_Y, const SIM_RawField * D, const SIM_RawField * T, fpreal t_amb, const UT_JobInfo &info);
)

#endif //HINAPE_GAS_HINA_GRIDEXTERNALFORCE_H
