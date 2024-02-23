#include "SIM_Hina_DFSPHParticles.h"
#include <CUDA_HinaPE/kernels.h>

SIM_HINA_DERIVED_GEOMETRY_CLASS_IMPLEMENT(
		DFSPHParticles,
		Particles,
		true,
)
void SIM_Hina_DFSPHParticles::_init_DFSPHParticles() {}
void SIM_Hina_DFSPHParticles::_makeEqual_DFSPHParticles(const SIM_Hina_DFSPHParticles *src) {}
void SIM_Hina_DFSPHParticles::_setup_gdp(GU_Detail *gdp) const
{
	SIM_Hina_Particles::_setup_gdp(gdp);
	HINA_GEOMETRY_POINT_ATTRIBUTE(HINA_GEOMETRY_ATTRIBUTE_DFSPH_ALPHA, HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
}
void SIM_Hina_DFSPHParticles::calculate_alpha()
{

}
