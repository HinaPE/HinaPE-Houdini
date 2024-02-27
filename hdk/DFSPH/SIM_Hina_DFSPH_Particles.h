#ifndef HINAPE_SIM_HINA_DFSPH_PARTICLES_H
#define HINAPE_SIM_HINA_DFSPH_PARTICLES_H

#include <Base/SIM_Hina_Particles.h>
#include <HinaPE/DFSPH/DFSPH.h>

SIM_HINA_DERIVED_GEOMETRY_CLASS(
		DFSPH_Particles,
		Particles,
		void commit() override;

		HinaPE::CPUScalarArray *alpha, *kappa_density, *kappa_divergence, *rho_adv, *d_rho;
)

#endif //HINAPE_SIM_HINA_DFSPH_PARTICLES_H
