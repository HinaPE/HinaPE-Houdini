#ifndef HINAPE_GAS_HINA_TEST_H
#define HINAPE_GAS_HINA_TEST_H

#include <SIM_Hina_Generator.h>

#include "common/geometry.h"
using real = float;
using Vector = UT_Vector3;
using Quaternion = UT_Quaternion;
using Surface = HinaPE::ISurface<real, Vector, Quaternion>;
using SurfacePtr = std::shared_ptr<Surface>;

GAS_HINA_SUBSOLVER_CLASS(
		Test,

		SurfacePtr InnerPtr;
)

#endif //HINAPE_GAS_HINA_TEST_H
