#include "solver.h"

SIM_Solver::SIM_Result Solver::solveSingleObjectSubclass(SIM_Engine &engine, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep, bool newobject)
{
	return SIM_SOLVER_SUCCESS;
}
