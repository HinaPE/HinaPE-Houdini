#include <UT/UT_DSOVersion.h> // Very Important!!! Include this!!!

#include "data.h"
#include "solver.h"

void
initializeSIM(void *)
{
	IMPLEMENT_DATAFACTORY(HinaClothSolver);
	IMPLEMENT_DATAFACTORY(HinaClothData);
}
