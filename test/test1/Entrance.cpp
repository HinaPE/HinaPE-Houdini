#include <UT/UT_DSOVersion.h> // Very Important!!! Include this!!!

#include "SIM_PBF.h"

void initializeSIM(void *)
{
	IMPLEMENT_DATAFACTORY(SIM_PBFSolver);
}
