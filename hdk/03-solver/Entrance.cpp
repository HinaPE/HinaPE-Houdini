#include <UT/UT_DSOVersion.h> // Very Important!!! Include this!!!

#include "data.h"
#include "solver.h"
#include "collider.h"

void
initializeSIM(void *)
{
//	IMPLEMENT_DATAFACTORY(HinaClothSolver);
	IMPLEMENT_DATAFACTORY(HinaClothSolver2);
	IMPLEMENT_DATAFACTORY(HinaClothData)
	IMPLEMENT_DATAFACTORY(HinaCollider)
}
