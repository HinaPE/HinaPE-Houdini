#ifndef HINAPE_GAS_HINA_UPDATEDENSITYAKINCI_H
#define HINAPE_GAS_HINA_UPDATEDENSITYAKINCI_H

#include <SIM_Hina_Generator.h>

class SIM_Hina_Particles;
class SIM_Hina_Akinci2012BoundaryParticles;

GAS_HINA_SUBSOLVER_CLASS(
		UpdateDensityAkinci,
		HINA_GETSET_PARAMETER(Kernel, GETSET_DATA_FUNCS_I)

		static void calculate_density(SIM_Hina_Particles *particles, std::map<UT_String, SIM_Hina_Akinci2012BoundaryParticles *> &akinci_boundaries);
)

#endif //HINAPE_GAS_HINA_UPDATEDENSITYAKINCI_H
