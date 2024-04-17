//
// Created by LiYifan on 2024/3/5.
//

#include "SIM_Hina_Particles_PBF.h"

SIM_HINA_DERIVED_GEOMETRY_CLASS_IMPLEMENT(
        Particles_PBF,
        Particles,
        true,
)

void SIM_Hina_Particles_PBF::_init_Particles_PBF()
{
    this->lambda = nullptr;
    this->pred_x = nullptr;
    this->delta_p = nullptr;
    this->a_ext = nullptr;
}

void SIM_Hina_Particles_PBF::_makeEqual_Particles_PBF(const SIM_Hina_Particles_PBF *src)
{
    this->lambda = src->lambda;
    this->pred_x = src->pred_x;
    this->delta_p = src->delta_p;
    this->a_ext = src->a_ext;
}

void SIM_Hina_Particles_PBF::_setup_gdp(GU_Detail *gdp) const
{
    SIM_Hina_Particles::_setup_gdp(gdp);
    HINA_GEOMETRY_POINT_ATTRIBUTE("pbf_lambda", HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
    HINA_GEOMETRY_POINT_ATTRIBUTE("pbf_pred_x", HINA_GEOMETRY_ATTRIBUTE_TYPE_VECTOR3)
    HINA_GEOMETRY_POINT_ATTRIBUTE("pbf_delta_p", HINA_GEOMETRY_ATTRIBUTE_TYPE_VECTOR3)
    HINA_GEOMETRY_POINT_ATTRIBUTE("pbf_a_ext", HINA_GEOMETRY_ATTRIBUTE_TYPE_VECTOR3)
}

void SIM_Hina_Particles_PBF::commit()
{
    SIM_Hina_Particles::commit();

    if (lambda == nullptr || pred_x == nullptr || delta_p == nullptr || a_ext == nullptr)
    {
        std::cout << "SIM_Hina_Particles_PBF::load() called with nullptr" << std::endl;
        return;
    }

    /*size_t size = lambda->size();
    if (size == 0)
        return;*/

    SIM_GeometryAutoWriteLock lock(this);
    GU_Detail &gdp = lock.getGdp();
    GA_RWHandleF lambda_handle = gdp.findPointAttribute("pbf_lambda");
    GA_RWHandleV3 pred_x_handle = gdp.findPointAttribute("pbf_pred_x");
    GA_RWHandleV3 delta_p_handle = gdp.findPointAttribute("pbf_delta_p");
    GA_RWHandleV3 a_ext_handle = gdp.findPointAttribute("pbf_a_ext");

    GA_Offset pt_off;
    GA_FOR_ALL_PTOFF(&gdp, pt_off)
        {
            fpreal l = (*lambda)[offset2index[pt_off]];
            UT_Vector3 predX = (*pred_x)[offset2index[pt_off]];
            UT_Vector3 deltaP = (*delta_p)[offset2index[pt_off]];
            UT_Vector3 aExt = (*a_ext)[offset2index[pt_off]];
            lambda_handle.set(pt_off, l);
            pred_x_handle.set(pt_off, predX);
            delta_p_handle.set(pt_off, deltaP);
            a_ext_handle.set(pt_off, aExt);
        }
}