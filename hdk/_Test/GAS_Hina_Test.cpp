#include "GAS_Hina_Test.h"

#include <Smoke/FieldUtils.h>
#include <CE/CE_Grid.h>
#include <iostream>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		Test,
		true,
		false,
		ACTIVATE_GAS_VELOCITY \
        ACTIVATE_GAS_DENSITY \
		ACTIVATE_GAS_VELOCITY2 \
        ACTIVATE_GAS_DENSITY2 \
)
void GAS_Hina_Test::_init() {}
void GAS_Hina_Test::_makeEqual(const GAS_Hina_Test *src) {}
bool GAS_Hina_Test::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_ScalarField *D = getScalarField(obj, GAS_NAME_DENSITY);
	SIM_VectorField *V = getVectorField(obj, GAS_NAME_VELOCITY);
	SIM_ScalarField *D2 = getScalarField(obj, GAS_NAME_DENSITY2);
	SIM_VectorField *V2 = getVectorField(obj, GAS_NAME_VELOCITY2);

	if (!D || !V || !D2 || !V2)
		return false;

	{
		auto &Field1 = *D;
		if (Field1.getVoxelSample() == SIM_SAMPLE_CENTER)
		{
			CubbyFlow::CellCenteredScalarGrid3Ptr Field2 = CubbyFlow::CellCenteredScalarGrid3::GetBuilder().MakeShared();
			HinaPE::ToCubby(Field1, Field2);
			HinaPE::print(Field1);
			HinaPE::print(Field2);
			HinaPE::match(Field1, Field2);

			HinaPE::ToHDK(Field2, *D2);
			std::cout << "The Same D - D2: " << D->getField()->isMatching(D2->getField()) << std::endl;
		} else if (Field1.getVoxelSample() == SIM_SAMPLE_CORNER)
		{
			CubbyFlow::VertexCenteredScalarGrid3Ptr Field2 = CubbyFlow::VertexCenteredScalarGrid3::GetBuilder().MakeShared();
			HinaPE::ToCubby(Field1, Field2);
			HinaPE::print(Field1);
			HinaPE::print(Field2);
			HinaPE::match(Field1, Field2);

			HinaPE::ToHDK(Field2, *D2);
			std::cout << "The Same D - D2: " << D->getField()->isMatching(D2->getField()) << std::endl;
		}
	}

	{
		auto &Field1 = *V;
		if (V->isFaceSampled())
		{
			CubbyFlow::FaceCenteredGrid3Ptr Field2 = CubbyFlow::FaceCenteredGrid3 ::GetBuilder().MakeShared();
			HinaPE::ToCubby(Field1, Field2);
			HinaPE::print(Field1);
			HinaPE::print(Field2);
			HinaPE::match(Field1, Field2);

			std::cout << "The Same V - V2: "
					<< V->getXField()->isMatching(V2->getXField()) << ", "
					<< V->getYField()->isMatching(V2->getYField()) << ", "
					<< V->getZField()->isMatching(V2->getZField()) << ", "
					<< std::endl;
		}
	}

	return true;
}