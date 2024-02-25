#ifndef HINAPE_GAS_HINA_CLEARFORCE_H
#define HINAPE_GAS_HINA_CLEARFORCE_H

#include <SIM_Hina_Generator.h>

class SIM_Hina_Particles;

GAS_HINA_SUBSOLVER_CLASS(
		ClearForce,

		void calculate_clear_force(SIM_Hina_Particles *particles);
)

#endif //HINAPE_GAS_HINA_CLEARFORCE_H
