#ifndef HINAPE_GAS_HINA_TEST_H
#define HINAPE_GAS_HINA_TEST_H

#include <SIM_Hina_Generator.h>

#include "common/geometry.h"
using Surface = HinaPE::ISurface<fpreal32, UT_Vector3, UT_Quaternion>;
using SurfacePtr = std::shared_ptr<Surface>;

GAS_HINA_SUBSOLVER_CLASS(
		Test,

		SurfacePtr InnerPtr;
)

#endif //HINAPE_GAS_HINA_TEST_H
