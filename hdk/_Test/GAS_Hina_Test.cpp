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
//	if (!V->isFaceSampled())
//		return false;

	{
		const SIM_RawField &Field = *D->getField();
		auto res = HinaPE::ToCubby(Field);

		std::cout << "Field.getVoxelRes(): " << Field.getVoxelRes() << std::endl;
		std::cout << "Field.getVoxelSize(): " << Field.getVoxelSize() << std::endl;
		std::cout << "Field.getOrig(): " << Field.getOrig() << std::endl;
	}
	return true;
}
