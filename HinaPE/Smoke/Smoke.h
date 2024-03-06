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
namespace HinaPE
{
using real = float;
using Vector = UT_Vector3T<real>;
using VectorField = SIM_VectorField;
using ScalarField = SIM_ScalarField;

struct SmokeField
{
	VectorField *V;
	ScalarField *D;
	ScalarField *T;
};

struct SmokeSolver
{
	SmokeSolver(VectorField *V, ScalarField *D, ScalarField *T);
	virtual void Solve(real dt);
	std::shared_ptr<SmokeField> Smoke;

protected:
	void advect();
};
}
#endif //HINAPE_SMOKE_H
