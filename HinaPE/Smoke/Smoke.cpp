#include "Smoke.h"

#include <memory>

HinaPE::SmokeSolver::SmokeSolver(HinaPE::VectorField *V, HinaPE::ScalarField *D, HinaPE::ScalarField *T)
{
	Smoke = std::make_shared<SmokeField>();
	Smoke->V = V;
	Smoke->D = D;
	Smoke->T = T;
}
void HinaPE::SmokeSolver::Solve(HinaPE::real dt)
{
}
void HinaPE::SmokeSolver::advect()
{

}
