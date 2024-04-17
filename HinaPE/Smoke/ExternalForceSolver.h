#ifndef HINAPE_EXTERNALFORCESOLVER_H
#define HINAPE_EXTERNALFORCESOLVER_H

#include <SIM/SIM_RawField.h>

namespace HinaPE
{
struct ExternalForceParam
{
	float GRAVITY;
	float DENSITY_FACTOR;
	float TEMPERATURE_FACTOR;
};

struct ExternalForceSolver : public ExternalForceParam
{
	void solve(const float dt, SIM_RawField *IO_V_Y, const SIM_RawField *IN_D, const SIM_RawField *IN_T);

private:
	THREADED_METHOD2(ExternalForceSolver, V_Y->shouldMultiThread(), _apply_gravity, float, dt, SIM_RawField *, V_Y);
	void _apply_gravityPartial(float dt, SIM_RawField *V_Y, const UT_JobInfo &info);

	THREADED_METHOD5(ExternalForceSolver, V_Y->shouldMultiThread(), _apply_buoyancy, float, dt, SIM_RawField *, V_Y, const SIM_RawField *, D, const SIM_RawField *, T, fpreal, t_amb);
	void _apply_buoyancyPartial(float dt, SIM_RawField *V_Y, const SIM_RawField *D, const SIM_RawField *T, fpreal t_amb, const UT_JobInfo &info);
};
}

#endif //HINAPE_EXTERNALFORCESOLVER_H
