#include <OP/OP_OperatorTable.h>
#include <OP/OP_Operator.h>
#include <DOP/DOP_Operator.h>
#include <UT/UT_DSOVersion.h>
#include <SIM/SIM_Utils.h>
#include "solver.h"

void initializeSIM(void*)
{
    IMPLEMENT_DATAFACTORY(Solver);
}
