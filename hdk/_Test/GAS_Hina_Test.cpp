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
	SIM_VectorField *V = getVectorField(obj, GAS_NAME_VELOCITY);

	if (!D || !V)
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
		} else if (Field1.getVoxelSample() == SIM_SAMPLE_CORNER)
		{
			CubbyFlow::VertexCenteredScalarGrid3Ptr Field2 = CubbyFlow::VertexCenteredScalarGrid3::GetBuilder().MakeShared();
			HinaPE::ToCubby(Field1, Field2);
			HinaPE::print(Field1);
			HinaPE::print(Field2);
			HinaPE::match(Field1, Field2);
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
		}
	}

	return true;
}