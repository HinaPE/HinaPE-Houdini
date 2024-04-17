#ifndef HINAPE_DIFFUSIONSOLVER_H
#define HINAPE_DIFFUSIONSOLVER_H

#include <SIM/SIM_RawField.h>

namespace HinaPE
{
struct DiffusionParam
{
	float DIFFUSION = 0.0f;
};
struct DiffusionSolver : public DiffusionParam
{

};
}

#endif //HINAPE_DIFFUSIONSOLVER_H
