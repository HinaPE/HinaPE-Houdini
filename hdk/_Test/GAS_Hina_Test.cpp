#include "GAS_Hina_Test.h"

#include <Smoke/FieldUtils.h>
#include <CE/CE_Grid.h>
#include <iostream>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		Test,
		true,
		false,
		ACTIVATE_GAS_VELOCITY \
        ACTIVATE_GAS_DENSITY
)
void GAS_Hina_Test::_init() {}
void GAS_Hina_Test::_makeEqual(const GAS_Hina_Test *src) {}
bool GAS_Hina_Test::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_ScalarField *D = getScalarField(obj, GAS_NAME_DENSITY);
//	SIM_VectorField *V = getVectorField(obj, GAS_NAME_VELOCITY);
//
//	if (!D || !V)
//		return false;
//
//	if (!V->isFaceSampled())
//		return false;

	{
		const SIM_RawField &Field = *D->getField();
		auto FieldCubby = HinaPE::ToCubby(Field);
		auto Field2 = HinaPE::ToHDK(FieldCubby);

		std::cout << "match1: " << HinaPE::match(Field, *FieldCubby) << std::endl;
		std::cout << "match2: " << HinaPE::match(Field2, *FieldCubby) << std::endl;
	}

	return true;
}