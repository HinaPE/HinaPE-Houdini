#include "GAS_Hina_GridBoundarySolver.h"

#include <HinaPE/Smoke/BoundarySolver.h>

enum GridType
{
	AIR = 0,
	BOUNDARY = 1,
	FLUID = 2,
};

GAS_HINA_SUBSOLVER_IMPLEMENT(
		GridBoundarySolver,
		true,
		false,
		HINA_INT_PARAMETER(Iter, 4) \
        ACTIVATE_GAS_SOURCE \
        ACTIVATE_GAS_DENSITY \
        ACTIVATE_GAS_TEMPERATURE \
        ACTIVATE_GAS_COLLISION \
        ACTIVATE_GAS_VELOCITY \
)

void GAS_Hina_GridBoundarySolver::_init() {}
void GAS_Hina_GridBoundarySolver::_makeEqual(const GAS_Hina_GridBoundarySolver *src) {}
bool GAS_Hina_GridBoundarySolver::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_ScalarField *S = getScalarField(obj, GAS_NAME_SOURCE);
	SIM_ScalarField *D = getScalarField(obj, GAS_NAME_DENSITY);
	SIM_ScalarField *T = getScalarField(obj, GAS_NAME_TEMPERATURE);
	SIM_ScalarField *C = getScalarField(obj, GAS_NAME_COLLISION);
	SIM_VectorField *V = getVectorField(obj, GAS_NAME_VELOCITY);

	if (!S || !D || !T || !C || !V)
		return false;

	if (!V->isFaceSampled())
		return false;

	static HinaPE::BoundarySolver _;
	SIM_RawField MarkerX0;
	SIM_RawField MarkerY0;
	SIM_RawField MarkerZ0;
	_._build_marker(&MarkerX0, V->getXField(), C->getField(), UT_Axis3::XAXIS);
	_._build_marker(&MarkerY0, V->getYField(), C->getField(), UT_Axis3::YAXIS);
	_._build_marker(&MarkerZ0, V->getZField(), C->getField(), UT_Axis3::ZAXIS);
	SIM_RawField MarkerX1 = MarkerX0;
	SIM_RawField MarkerY1 = MarkerY0;
	SIM_RawField MarkerZ1 = MarkerZ0;
	for (int i = 0; i < getIter(); ++i)
	{
		if (i % 2)
		{
			_._extrapolate(V->getXField(), &MarkerX1, &MarkerX0);
			_._extrapolate(V->getYField(), &MarkerY1, &MarkerY0);
			_._extrapolate(V->getZField(), &MarkerZ1, &MarkerZ0);
		} else
		{
			_._extrapolate(V->getXField(), &MarkerX0, &MarkerX1);
			_._extrapolate(V->getYField(), &MarkerY0, &MarkerY1);
			_._extrapolate(V->getZField(), &MarkerZ0, &MarkerZ1);
		}
	}
	SIM_RawField VEL_X0 = *V->getXField();
	SIM_RawField VEL_Y0 = *V->getYField();
	SIM_RawField VEL_Z0 = *V->getZField();
	_._fractional(V->getXField(), &VEL_X0, &VEL_Y0, &VEL_Z0, C->getField(), UT_Axis3::XAXIS);
	_._fractional(V->getYField(), &VEL_Y0, &VEL_X0, &VEL_Z0, C->getField(), UT_Axis3::YAXIS);
	_._fractional(V->getZField(), &VEL_Z0, &VEL_X0, &VEL_Y0, C->getField(), UT_Axis3::ZAXIS);
	_._enforce_boundary(V->getXField(), V->getYField(), V->getZField());

	return true;
}
