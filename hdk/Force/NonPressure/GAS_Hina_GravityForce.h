#ifndef HINAPE_GAS_HINA_GRAVITYFORCE_H
#define HINAPE_GAS_HINA_GRAVITYFORCE_H

#include <SIM_Hina_Generator.h>

class SIM_Hina_Particles;
class SIM_Hina_Akinci2012BoundaryParticles;

GAS_HINA_SUBSOLVER_CLASS(
		GravityForce,
		HINA_GETSET_PARAMETER(Gravity, GETSET_DATA_FUNCS_V3)

		void calculate_gravity_force(SIM_Hina_Particles *particles);
)

#endif //HINAPE_GAS_HINA_GRAVITYFORCE_H
