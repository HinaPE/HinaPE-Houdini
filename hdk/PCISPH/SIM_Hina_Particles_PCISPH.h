//
// Created by LiYifan on 2024/4/18.
//

#ifndef HINAPE_HOUDINI_SIM_HINA_PARTICLES_PCISPH_H
#define HINAPE_HOUDINI_SIM_HINA_PARTICLES_PCISPH_H

#include <Base/SIM_Hina_Particles.h>

SIM_HINA_DERIVED_GEOMETRY_CLASS(
        Particles_PCISPH,
        Particles,
        void commit() override;

        ScalarArrayCPU *pred_density, *pressure, *d_error;
        VectorArrayCPU *pred_x, *pred_v, *a_ext , *a_pressure;
)

#endif //HINAPE_HOUDINI_SIM_HINA_PARTICLES_PCISPH_H
