#include "GAS_Hina_UpdateDensity.h"
#include <Particles/SIM_Hina_Particles.h>
#include <CUDA_CubbyFlow/Core/Particle/SPHKernels.hpp>
#include <CUDA_HinaPE/kernels.h>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		UpdateDensity,
		true,
		false,
		TARGET_PARTICLE_GEOMETRY(SIM_Hina_Particles)
		static std::array<PRM_Name, 4> Kernels = { \
            PRM_Name("0", "Poly64"), \
            PRM_Name("1", "Spiky"), \
            PRM_Name("2", "CubicSpline"), \
            PRM_Name(nullptr), \
		}; \
        static PRM_Name KernelName("Kernel", "Kernel"); \
        static PRM_Default KernelNameDefault(2, "CubicSpline"); \
        static PRM_ChoiceList CL(PRM_CHOICELIST_SINGLE, Kernels.data()); \
        PRMS.emplace_back(PRM_ORD, 1, &KernelName, &KernelNameDefault, &CL); \
)

void GAS_Hina_UpdateDensity::_init() {}
void GAS_Hina_UpdateDensity::_makeEqual(const GAS_Hina_UpdateDensity *src) {}
bool GAS_Hina_UpdateDensity::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_Hina_Particles *particles = SIM_DATA_CAST(getGeometryCopy(obj, GAS_NAME_GEOMETRY), SIM_Hina_Particles);
	CHECK_NULL_RETURN_BOOL(particles)

	particles->calculate_density();
	return true;
}
