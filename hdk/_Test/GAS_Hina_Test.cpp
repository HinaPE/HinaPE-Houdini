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

//	{
//		const SIM_RawField &Field = *D->getField();
//		auto res = HinaPE::ToCubby(Field);
//		auto res2 = HinaPE::ToHDK(res);
//
//		std::cout << "Field.isMatching(&res2)" << Field.isMatching(&res2) << std::endl;
//		std::cout << "Field.indexToPos({0, 0, 0})"
//				  << Field.indexToPos({0, 0, 0})
//				  << "(*res).DataPosition()(0, 0, 0)"
//				  << (*res).DataPosition()(0, 0, 0).x << ", "
//				  << (*res).DataPosition()(0, 0, 0).y << ", "
//				  << (*res).DataPosition()(0, 0, 0).z
//				  << std::endl;
//
//		std::cout << "Field.getVoxelRes(): " << Field.getVoxelRes() << std::endl;
//		std::cout << "Field.getVoxelSize(): " << Field.getVoxelSize() << std::endl;
//		std::cout << "Field.getOrig(): " << Field.getOrig() << std::endl;
//	}

	{
		SIM_RawField t1, t2;
		t1.init(SIM_SAMPLE_CENTER, UT_Vector3(0, 0, 0), UT_Vector3(1, 1, 1), 10, 10, 10);
		HinaPE::print(t1);
		std::cout << " ================== " << std::endl;
		t2.init(SIM_SAMPLE_CORNER, UT_Vector3(0, 0, 0), UT_Vector3(1, 1, 1), 10, 10, 10);
		HinaPE::print(t2);

		std::cout << "matching? " << t1.isMatching(&t2) << std::endl; // true
		std::cout << "aligned? " << t1.isAligned(&t2) << std::endl; // false
	}
	return true;
}
