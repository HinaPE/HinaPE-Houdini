#include "GAS_Hina_ShowInfo.h"

#include <HinaPE/Smoke/FieldUtils.h>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		ShowInfo,
		true,
		false,

		ACTIVATE_GAS_VELOCITY \
        ACTIVATE_GAS_DENSITY \
        ACTIVATE_GAS_TEMPERATURE \
        ACTIVATE_GAS_COLLISION \
)

void GAS_Hina_ShowInfo::_init() {}
void GAS_Hina_ShowInfo::_makeEqual(const GAS_Hina_ShowInfo *src) {}
bool GAS_Hina_ShowInfo::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_ScalarField *D = getScalarField(obj, GAS_NAME_DENSITY);
	SIM_ScalarField *T = getScalarField(obj, GAS_NAME_TEMPERATURE);
	SIM_ScalarField *C = getScalarField(obj, GAS_NAME_COLLISION);
	SIM_VectorField *V = getVectorField(obj, GAS_NAME_VELOCITY);

	if (D)
	{
		HinaPE::print(*D);
		HinaPE::print(*(D->getField()));
	}

	if (T)
	{
		HinaPE::print(*T);
		HinaPE::print(*(T->getField()));
	}

	if (C)
	{
		HinaPE::print(*C);
		HinaPE::print(*(C->getField()));
	}

	if (V)
	{
		HinaPE::print(*V);
		HinaPE::print(*(V->getXField()));
		HinaPE::print(*(V->getYField()));
		HinaPE::print(*(V->getZField()));
	}

	return true;
}
