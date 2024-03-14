#ifndef HINAPE_SMOKE_H
#define HINAPE_SMOKE_H

/**
 * Grid Based Smoke Simulation
 *
 * NOT IMPLEMENTED YET
 */

#include <vector>
#include <UT/UT_Vector3.h>
#include <SIM/SIM_ScalarField.h>
#include <SIM/SIM_VectorField.h>
#include "common/geometry.h"
namespace HinaPE
{
using Surface = HinaPE::ISurface<fpreal32, UT_Vector3, UT_Quaternion>;

struct SmokeNativeSolver
{
	static void emit(SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V);
	static void apply_external_forces(float dt, SIM_VectorField *V);
	static void apply_viscosity(float dt);
	static void apply_pressure(float dt);
	static void apply_advection(float dt, SIM_ScalarField *SF, const SIM_VectorField *V);
};
}
#endif //HINAPE_SMOKE_H
