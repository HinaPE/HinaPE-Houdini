#ifndef HINAPE_SIM_HINA_PARTICLES_H
#define HINAPE_SIM_HINA_PARTICLES_H

#include <SIM_Hina_Generator.h>

using real = float;
using Vector = UT_Vector3T<real>;
using ScalarArrayCPU = std::vector<real>;
using VectorArrayCPU = std::vector<Vector>;

SIM_HINA_GEOMETRY_CLASS(
		Particles,

		bool gdp_dirty;
		std::map<GA_Offset, GA_Size> offset2index;
		std::map<GA_Size, GA_Offset> index2offset;
//		virtual void load();
		virtual void commit(); // auto commit by `GAS_CFL_SubStep`

		VectorArrayCPU *x, *v, *a;
		ScalarArrayCPU *m, *V, *rho, *nt, *no;
)

#endif //HINAPE_SIM_HINA_PARTICLES_H
