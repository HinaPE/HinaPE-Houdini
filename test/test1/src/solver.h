#ifndef SIM_PBF_SOLVER_H
#define SIM_PBF_SOLVER_H

#include "utils.h"

#include <GU/GU_Detail.h>
#include <SIM/SIM_PointNeighbourList.h>
#include <UT/UT_ParallelUtil.h>

//void InitPBFData(HinaPE_PBF::PBF_DATA &data, const GU_Detail &gdp);



template<typename SRC>
void BuildNeighbourList_Native(
		UT_Array<UT_Array<GA_Offset>> &OutNeighbourList, const UT_Vector3TArrayHandle<SRC> &InPoints, sz InPointsSize, sz StartIndex = 0)
{

}
#endif //SIM_PBF_SOLVER_H
