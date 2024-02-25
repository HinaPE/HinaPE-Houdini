#ifndef HINAPE_GAS_HINA_DFSPHSOLVER_H
#define HINAPE_GAS_HINA_DFSPHSOLVER_H

#include <SIM_Hina_Generator.h>

class SIM_Hina_DFSPHParticles;

class SIM_Hina_Akinci2012BoundaryParticles;

GAS_HINA_SUBSOLVER_CLASS(
		DFSPHSolver,

		fpreal calculate_CFL_dt(SIM_Hina_DFSPHParticles *fluid, fpreal t_max);
		void calculate_non_pressure_force(SIM_Hina_DFSPHParticles *fluid);
		void calculate_alpha(SIM_Hina_DFSPHParticles *fluid, std::map<UT_String, SIM_Hina_Akinci2012BoundaryParticles *> &akinci_boundaries);
		void correct_density_error(SIM_Hina_DFSPHParticles *fluid, std::map<UT_String, SIM_Hina_Akinci2012BoundaryParticles *> &akinci_boundaries, fpreal dt);
		void correct_divergence_error(SIM_Hina_DFSPHParticles *fluid, std::map<UT_String, SIM_Hina_Akinci2012BoundaryParticles *> &akinci_boundaries, fpreal dt);

private:
		void _compute_density_change(SIM_Hina_DFSPHParticles *fluid, std::map<UT_String, SIM_Hina_Akinci2012BoundaryParticles *> &akinci_boundaries, fpreal dt);
		fpreal avg_density_adv;
		fpreal avg_derived_density;
)

#endif //HINAPE_GAS_HINA_DFSPHSOLVER_H