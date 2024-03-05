#include "GAS_Hina_Solver_Smoke.h"

GAS_HINA_SUBSOLVER_IMPLEMENT(
		Solver_Smoke,
		true,
		false,

		static PRM_Name theDstFieldName(GAS_NAME_FIELDDEST, "Dest Field"); \
        static PRM_Name theSrcFieldName(GAS_NAME_FIELDSOURCE, "Source Field"); \
        PRMS.emplace_back(PRM_STRING, 1, &theDstFieldName); \
        PRMS.emplace_back(PRM_STRING, 1, &theSrcFieldName); \
)

void GAS_Hina_Solver_Smoke::_init() {}
void GAS_Hina_Solver_Smoke::_makeEqual(const GAS_Hina_Solver_Smoke *src) {}
bool GAS_Hina_Solver_Smoke::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_DataArray src, dst;
	getMatchingData(src, obj, GAS_NAME_FIELDSOURCE);
	getMatchingData(dst, obj, GAS_NAME_FIELDDEST);

	for (GA_Size i = 0; i < dst.entries(); i++)
	{
		SIM_ScalarField *srcscalar = SIM_DATA_CAST(src(i), SIM_ScalarField);
		SIM_ScalarField *dstscalar = SIM_DATA_CAST(dst(i), SIM_ScalarField);

		if (srcscalar && dstscalar)
		{
			std::cout << "SCALAR SRC Size: " << srcscalar->getSize() << std::endl;
			std::cout << "SCALAR DST Size: " << dstscalar->getSize() << std::endl;
		}

		SIM_VectorField *srcvector = SIM_DATA_CAST(src(i), SIM_VectorField);
		SIM_VectorField *dstvector = SIM_DATA_CAST(dst(i), SIM_VectorField);

		if (srcvector && dstvector)
		{
			std::cout << "VECTOR SRC Size: " << srcvector->getSize() << std::endl;
			std::cout << "VECTOR DST Size: " << dstvector->getSize() << std::endl;
		}
	}
	return true;
}
