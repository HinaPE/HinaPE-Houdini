//
// Created by LiYifan on 2024/4/18.
//

#include "SIM_Hina_Particles_PCISPH.h"

SIM_HINA_DERIVED_GEOMETRY_CLASS_IMPLEMENT(
        Particles_PCISPH,
        Particles,
        true,
)

void SIM_Hina_Particles_PCISPH::_init_Particles_PCISPH() {
    this->pred_density = nullptr;
    this->pressure = nullptr;
    this->d_error = nullptr;
    this->pred_x = nullptr;
    this->pred_v = nullptr;
    this->a_ext = nullptr;
    this->a_pressure = nullptr;
    this->delta = nullptr;
}

void SIM_Hina_Particles_PCISPH::_makeEqual_Particles_PCISPH(const SIM_Hina_Particles_PCISPH *src) {
    this->pred_density = src->pred_density;
    this->pressure = src->pressure;
    this->d_error = src->d_error;
    this->pred_x = src->pred_x;
    this->pred_v = src->pred_v;
    this->a_ext = src->a_ext;
    this->a_pressure = src->a_pressure;
    this->delta = src->delta;
}

void SIM_Hina_Particles_PCISPH::_setup_gdp(GU_Detail *gdp) const {
    SIM_Hina_Particles::_setup_gdp(gdp);
    HINA_GEOMETRY_POINT_ATTRIBUTE("pred_density", HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
    HINA_GEOMETRY_POINT_ATTRIBUTE("pressure", HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
    HINA_GEOMETRY_POINT_ATTRIBUTE("d_error", HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
    HINA_GEOMETRY_POINT_ATTRIBUTE("pred_x", HINA_GEOMETRY_ATTRIBUTE_TYPE_VECTOR3)
    HINA_GEOMETRY_POINT_ATTRIBUTE("pred_v", HINA_GEOMETRY_ATTRIBUTE_TYPE_VECTOR3)
    HINA_GEOMETRY_POINT_ATTRIBUTE("a_ext", HINA_GEOMETRY_ATTRIBUTE_TYPE_VECTOR3)
    HINA_GEOMETRY_POINT_ATTRIBUTE("a_pressure", HINA_GEOMETRY_ATTRIBUTE_TYPE_VECTOR3)
    HINA_GEOMETRY_POINT_ATTRIBUTE("delta", HINA_GEOMETRY_ATTRIBUTE_TYPE_FLOAT)
}

void SIM_Hina_Particles_PCISPH::commit() {
    SIM_Hina_Particles::commit();

    if (pred_density == nullptr || pressure == nullptr || d_error == nullptr || pred_x == nullptr || pred_v == nullptr || a_ext == nullptr || a_pressure == nullptr) {
        std::cout << "SIM_Hina_Particles_PCISPH::load() called with nullptr" << std::endl;
        return;
    }

    SIM_GeometryAutoWriteLock lock(this);
    GU_Detail &gdp = lock.getGdp();
    GA_RWHandleF pred_density_handle = gdp.findPointAttribute("pred_density");
    GA_RWHandleF pressure_handle = gdp.findPointAttribute("pressure");
    GA_RWHandleF d_error_handle = gdp.findPointAttribute("d_error");
    GA_RWHandleV3 pred_x_handle = gdp.findPointAttribute("pred_x");
    GA_RWHandleV3 pred_v_handle = gdp.findPointAttribute("pred_v");
    GA_RWHandleV3 a_ext_handle = gdp.findPointAttribute("a_ext");
    GA_RWHandleV3 a_pressure_handle = gdp.findPointAttribute("a_pressure");
    GA_RWHandleF delta_handle = gdp.findPointAttribute("delta");

    GA_Offset pt_off;
    GA_FOR_ALL_PTOFF(&gdp, pt_off)
    {
        real pd = (*pred_density)[offset2index[pt_off]];
        real p = (*pressure)[offset2index[pt_off]];
        real de = (*d_error)[offset2index[pt_off]];
        UT_Vector3 px = (*pred_x)[offset2index[pt_off]];
        UT_Vector3 pv = (*pred_v)[offset2index[pt_off]];
        UT_Vector3 ae = (*a_ext)[offset2index[pt_off]];
        UT_Vector3 ap = (*a_pressure)[offset2index[pt_off]];
        real d = (*delta)[offset2index[pt_off]];

        pred_density_handle.set(pt_off, pd);
        pressure_handle.set(pt_off, p);
        d_error_handle.set(pt_off, de);
        pred_x_handle.set(pt_off, px);
        pred_v_handle.set(pt_off, pv);
        a_ext_handle.set(pt_off, ae);
        a_pressure_handle.set(pt_off, ap);
        delta_handle.set(pt_off, d);
    }
}
