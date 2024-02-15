#include "SIM_Hina_Akinci2012BoundaryParticles.h"

SIM_HINA_DERIVED_GEOMETRY_CLASS_IMPLEMENT(
		Akinci2012BoundaryParticles,
		Particles,
		true,
)

void SIM_Hina_Akinci2012BoundaryParticles::_init_Akinci2012BoundaryParticles() {}
void SIM_Hina_Akinci2012BoundaryParticles::_makeEqual_Akinci2012BoundaryParticles(const SIM_Hina_Akinci2012BoundaryParticles *src) {}
void SIM_Hina_Akinci2012BoundaryParticles::_setup_gdp(GU_Detail *gdp) const {}
void SIM_Hina_Akinci2012BoundaryParticles::Commit()
{
	SIM_Hina_Particles::Commit();
}
