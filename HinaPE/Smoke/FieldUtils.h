#ifndef HINAPE_FIELDUTILS_H
#define HINAPE_FIELDUTILS_H

#include <UT/UT_Vector3.h>
#include <SIM/SIM_RawField.h>

namespace HinaPE
{
using real = float;
using Vector = UT_Vector3T<real>;
real Gradient(SIM_RawField &F, int x, int y, int z);
}

#endif //HINAPE_FIELDUTILS_H
