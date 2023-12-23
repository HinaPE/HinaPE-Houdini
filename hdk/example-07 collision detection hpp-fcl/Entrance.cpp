#include <UT/UT_DSOVersion.h> // Very Important!!! Include this!!!

#include "solver.h"

void initializeSIM(void *)
{
	IMPLEMENT_DATAFACTORY(HPPFCLCollisionSolver);
}
