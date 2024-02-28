//
// Created by LiYifan on 2024/2/26.
//

#include "SIM_FluidParticle.h"
#include "SIM_Hina_Generator.h"

#include <SIM/SIM_Engine.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_Position.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_GuideShared.h>

#include <PRM/PRM_Name.h>
#include <PRM/PRM_Template.h>
#include <PRM/PRM_Shared.h>
#include <PRM/PRM_Default.h>
#include <PRM/PRM_Utils.h>
#include <PRM/PRM_SpareData.h>
#include <PRM/PRM_ChoiceList.h>

void SIM_FluidParticle::initializeSubclass() {
    SIM_Data::initializeSubclass();

    Configured = false;
    this->pbfParticle = nullptr;
    this->offset2index.clear();
    this->index2offset.clear();
}

void SIM_FluidParticle::makeEqualSubclass(const SIM_Data *source) {
    SIM_Data::makeEqualSubclass(source);
    const SIM_FluidParticle *src = SIM_DATA_CASTCONST(source, SIM_FluidParticle);
    this->Configured = src->Configured;
    this->pbfParticle = src->pbfParticle;

    this->offset2index = src->offset2index;
    this->index2offset = src->index2offset;
}

const char *SIM_FluidParticle::DATANAME = "Fluid_Particle";
const SIM_DopDescription *SIM_FluidParticle::GetDescription() {
    static PRM_Name FluidDomain("FluidDomain", "FluidDomain");
    static std::array<PRM_Default, 3> FluidDomainDefault{2, 2, 2};

    static PRM_Name TargetDensity("TargetDensity", "TargetDensity");
    static PRM_Default TargetDensityDefault(1000.0f);

    static PRM_Name TargetSpacing("TargetSpacing", "TargetSpacing");
    static PRM_Default TargetSpacingDefault(0.02f);

    static PRM_Name KernelRadiusOverTargetSpacing("KernelRadiusOverTargetSpacing", "KernelRadiusOverTargetSpacing");
    static PRM_Default KernelRadiusOverTargetSpacingDefault(1.8f);

    static PRM_Name Gravity("Gravity", "Gravity");
    static std::array<PRM_Default, 3> GravityDefault{0, -9.8, 0};

    static PRM_Template myTemplate[] = {
        PRM_Template(PRM_XYZ_J, 3, &FluidDomain, FluidDomainDefault.data()),
        PRM_Template(PRM_FLT_J, 1, &TargetDensity, &TargetDensityDefault),
        PRM_Template(PRM_FLT_J, 1, &TargetSpacing, &TargetSpacingDefault),
        PRM_Template(PRM_FLT_J, 1, &KernelRadiusOverTargetSpacing, &KernelRadiusOverTargetSpacingDefault),
        PRM_Template(PRM_XYZ_J, 3, &Gravity, GravityDefault.data()),
        PRM_Template()
    };

    static PRM_Name ShowGuideGeometry(SIM_NAME_SHOWGUIDE, "ShowGuideGeometry");

    static PRM_Name DomainColor("DomainColor", "DomainColor");
    static std::array<PRM_Default, 3> DomainColorDefault{.0156356, 0, .5};

    static std::array<PRM_Template, 3> PRMS_GUIDE{
            PRM_Template(PRM_TOGGLE, 1, &ShowGuideGeometry, PRMoneDefaults),
            PRM_Template(PRM_RGBA, 3, &DomainColor, DomainColorDefault.data()),
            PRM_Template()
    };

    static SIM_DopDescription DESC(true,
                                  "Fluid_Particle",
                                  "Fluid_Particle",
                                   DATANAME,
                                   classname(),
                                  myTemplate);
    DESC.setGuideTemplates(PRMS_GUIDE.data());
    return &DESC;
}

void SIM_FluidParticle::setup_gdp(GU_Detail *gdp) {
    GA_RWAttributeRef velocity_ref = gdp->addFloatTuple(GA_ATTRIB_POINT, HINA_GEOMETRY_ATTRIBUTE_VELOCITY, 3,GA_Defaults(0));
    GA_RWAttributeRef force_ref = gdp->addFloatTuple(GA_ATTRIB_POINT, HINA_GEOMETRY_ATTRIBUTE_FORCE, 3,GA_Defaults(0));
    GA_RWAttributeRef density_ref = gdp->addFloatTuple(GA_ATTRIB_POINT, HINA_GEOMETRY_ATTRIBUTE_DENSITY, 1,GA_Defaults(0));
    GA_RWAttributeRef pressure_ref = gdp->addFloatTuple(GA_ATTRIB_POINT, HINA_GEOMETRY_ATTRIBUTE_PRESSURE, 1,GA_Defaults(0));
    GA_RWAttributeRef point_neighbor_list_ref = gdp->addIntArray(GA_ATTRIB_POINT, "pnl");
    GA_RWAttributeRef point_neighbor_sum_ref = gdp->addIntTuple(GA_ATTRIB_POINT, "pns",1,GA_Defaults(0));
    GA_RWAttributeRef triangle_neighbor_list_ref = gdp->addIntArray(GA_ATTRIB_PRIMITIVE, "tnl");
    GA_RWAttributeRef triangle_neighbor_sum_ref = gdp->addIntTuple(GA_ATTRIB_PRIMITIVE, "tns",1,GA_Defaults(0));
}

void SIM_FluidParticle::load(SIM_GeometryCopy *geo) {
    if (!geo) {
        std::cout << "SIM_FluidParticle::load: geo is nullptr" << std::endl;
        return;
    }
    {
        SIM_GeometryAutoWriteLock lock(geo);
        GU_Detail &gdp = lock.getGdp();
        GA_ROHandleV3 pos_handle = gdp.getP();
        GA_ROHandleV3 vel_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VELOCITY);
        GA_ROHandleV3 force_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_FORCE);
        GA_ROHandleF density_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DENSITY);
        GA_ROHandleF pressure_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_PRESSURE);
        GA_ROHandleIA point_neighbor_list_handle = gdp.findPointAttribute("pnl");
        GA_ROHandleIA triangle_neighbor_list_handle = gdp.findPrimitiveAttribute("tnl");

        {
            if (!pos_handle.isValid()) {
                std::cout << "SIM_FluidParticle::load: pos_handle is invalid" << std::endl;
                return;
            }
            if (!vel_handle.isValid()) {
                std::cout << "SIM_FluidParticle::load: vel_handle is invalid" << std::endl;
                return;
            }
            if (!force_handle.isValid()) {
                std::cout << "SIM_FluidParticle::load: force_handle is invalid" << std::endl;
                return;
            }
            if (!density_handle.isValid()) {
                std::cout << "SIM_FluidParticle::load: density_handle is invalid" << std::endl;
                return;
            }
            if (!pressure_handle.isValid()) {
                std::cout << "SIM_FluidParticle::load: pressure_handle is invalid" << std::endl;
                return;
            }
            if (!point_neighbor_list_handle.isValid()) {
                std::cout << "SIM_FluidParticle::load: point_neighbor_list_handle is invalid" << std::endl;
                return;
            }
            if (!triangle_neighbor_list_handle.isValid()) {
                std::cout << "SIM_FluidParticle::load: triangle_neighbor_list_handle is invalid" << std::endl;
                return;
            }
        }

        GA_Offset pt_off;
        GA_FOR_ALL_PTOFF(&gdp, pt_off)
        {
            GA_Size index = gdp.pointIndex(pt_off);
            UT_Vector3 pos = pos_handle.get(pt_off);
            UT_Vector3 vel = vel_handle.get(pt_off);
            UT_Vector3 force = force_handle.get(pt_off);
            float density = density_handle.get(pt_off);
            float pressure = pressure_handle.get(pt_off);

            offset2index[pt_off] = index;
            index2offset[index] = pt_off;
            pbfParticle->positions[pt_off] = pos;
            pbfParticle->velocities[pt_off] = vel;
            pbfParticle->forces[pt_off] = force;
            pbfParticle->densities[pt_off] = density;
            pbfParticle->pressures[pt_off] = pressure;

            UT_Int32Array pArray;
            point_neighbor_list_handle.get(pt_off, pArray);
            for (int i = 0; i < pArray.size(); i++) {
                GA_Offset neighbor_off = gdp.pointOffset(pArray(i));
                ParticleState neighbor_state{};
                neighbor_state.pt_off = neighbor_off;
                neighbor_state.pt_pos = pos_handle.get(neighbor_off);
                pbfParticle->point_neighbors[pt_off].push_back(neighbor_state);
            }

            /// TODO: triangle neighbor list
            /*UT_Int32Array tArray;
            triangle_neighbor_list_handle.get(pt_off, tArray);
            for (int i = 0; i < tArray.size(); i++) {
                GA_Offset neighbor_off = gdp.primitiveOffset(tArray(i));
                ParticleState neighbor_state1{};
                ParticleState neighbor_state2{};
                ParticleState neighbor_state3{};


            }*/
        }
    }
}

void SIM_FluidParticle::commit(SIM_GeometryCopy *geo) const {
    if (!geo) {
        std::cout << "SIM_FluidParticle::load: geo is nullptr" << std::endl;
        return;
    }
    {
        SIM_GeometryAutoWriteLock lock(geo);
        GU_Detail &gdp = lock.getGdp();
        GA_RWHandleV3 pos_handle = gdp.getP();
        GA_RWHandleV3 vel_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_VELOCITY);
        GA_RWHandleV3 force_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_FORCE);
        GA_RWHandleF density_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_DENSITY);
        GA_RWHandleF pressure_handle = gdp.findPointAttribute(HINA_GEOMETRY_ATTRIBUTE_PRESSURE);
        GA_RWHandleIA point_neighbor_list_handle = gdp.findPointAttribute("pnl");
        GA_RWHandleI point_neighbor_sum_handle = gdp.findPointAttribute("pns");
        GA_RWHandleIA triangle_neighbor_list_handle = gdp.findPrimitiveAttribute("tnl");
        GA_RWHandleI triangle_neighbor_sum_handle = gdp.findPrimitiveAttribute("tns");

        {
            if (!pos_handle.isValid()) {
                std::cout << "SIM_FluidParticle::load: pos_handle is invalid" << std::endl;
                return;
            }
            if (!vel_handle.isValid()) {
                std::cout << "SIM_FluidParticle::load: vel_handle is invalid" << std::endl;
                return;
            }
            if (!force_handle.isValid()) {
                std::cout << "SIM_FluidParticle::load: force_handle is invalid" << std::endl;
                return;
            }
            if (!density_handle.isValid()) {
                std::cout << "SIM_FluidParticle::load: density_handle is invalid" << std::endl;
                return;
            }
            if (!pressure_handle.isValid()) {
                std::cout << "SIM_FluidParticle::load: pressure_handle is invalid" << std::endl;
                return;
            }
            if (!point_neighbor_list_handle.isValid()) {
                std::cout << "SIM_FluidParticle::load: point_neighbor_list_handle is invalid" << std::endl;
                return;
            }
            if (!triangle_neighbor_list_handle.isValid()) {
                std::cout << "SIM_FluidParticle::load: triangle_neighbor_list_handle is invalid" << std::endl;
                return;
            }
        }

        GA_Offset pt_off;
        GA_FOR_ALL_PTOFF(&gdp, pt_off)
        {
            GA_Size index = gdp.pointIndex(pt_off);

            UT_Vector3 pos = pbfParticle->positions[pt_off];
            UT_Vector3 vel = pbfParticle->velocities[pt_off];
            UT_Vector3 force = pbfParticle->forces[pt_off];
            float density = pbfParticle->densities[pt_off];
            float pressure = pbfParticle->pressures[pt_off];

            pos_handle.set(pt_off, pos);
            vel_handle.set(pt_off, vel);
            force_handle.set(pt_off, force);
            density_handle.set(pt_off, density);
            pressure_handle.set(pt_off, pressure);

            UT_Int32Array pArray;
            for (auto &neighbor : pbfParticle->point_neighbors[pt_off]) {
                pArray.append(gdp.pointIndex(neighbor.pt_off));
            }

            /*// check
            for(int i = 0; i < pArray.size(); i++)
            {
                std::cout << "pArray[" << i << "] = " << pArray(i) << std::endl;
            }*/

            point_neighbor_list_handle.set(pt_off, pArray);
            point_neighbor_sum_handle.set(pt_off, pArray.size());

            /// TODO: triangle neighbor list
            /*UT_Int32Array tArray;
            for (auto &neighbor : pbfParticle->triangle_neighbors[pt_off]) {

            }*/
        }
    }
}

void SIM_FluidParticle::for_each_offset(const std::function<void(const GA_Offset &)> &func) {
    for (const auto &pair: offset2index)
        func(pair.first);
}

void SIM_FluidParticle::for_each_neighbor_self(const GA_Offset &pt_off,
                                               const std::function<void(const GA_Offset &, const UT_Vector3 &)> &func) const {
    const auto &neighbors = pbfParticle->point_neighbors[pt_off];
    for (const auto &neighbor: neighbors)
        func(neighbor.pt_off, neighbor.pt_pos);
}

void SIM_FluidParticle::force_keep_boundary() {
    UT_Vector3 domain = getFluidDomain();
    UT_Vector3 half_domain = domain * 0.5;
    for_each_offset(
        [&](GA_Offset pt_off)
        {
            UT_Vector3 &pos = pbfParticle->positions[pt_off];
            UT_Vector3 &vel = pbfParticle->velocities[pt_off];
            if (pos.x() < -half_domain.x())
            {
                pos.x() = -half_domain.x();
                vel.x() = 0;
            }
            if (pos.x() > half_domain.x())
            {
                pos.x() = half_domain.x();
                vel.x() = 0;
            }
            if (pos.y() < -half_domain.y())
            {
                pos.y() = -half_domain.y();
                vel.y() = 0;
            }
            if (pos.y() > half_domain.y())
            {
                pos.y() = half_domain.y();
                vel.y() = 0;
            }
            if (pos.z() < -half_domain.z())
            {
                pos.z() = -half_domain.z();
                vel.z() = 0;
            }
            if (pos.z() > half_domain.z())
            {
                pos.z() = half_domain.z();
                vel.z() = 0;
            }
        }
    );
}

SIM_Guide *SIM_FluidParticle::createGuideObjectSubclass() const {
    return new SIM_GuideShared(this, true);
}

void SIM_FluidParticle::buildGuideGeometrySubclass(const SIM_RootData &root, const SIM_Options &options,
                                                   const GU_DetailHandle &gdh, UT_DMatrix4 *xform,
                                                   const SIM_Time &t) const {
    if (gdh.isNull())
        return;

    if (!getShowGuideGeometry(options))
        return;

    UT_Vector3 color = getDomainColor(options);

    GU_DetailHandleAutoWriteLock gdl(gdh);
    GU_Detail *gdp = gdl.getGdp();
    gdp->clearAndDestroy();

    UT_Vector3 Center = UT_Vector3(0.);
    UT_Vector3 Extent = getFluidDomain();

    std::array<UT_Vector3, 8> vertices{};
    for (int i = 0; i < 8; i++)
    {
        vertices[i] = UT_Vector3(
                Center.x() + Extent.x() * ((i & 1) ? 0.5 : -0.5),
                Center.y() + Extent.y() * ((i & 2) ? 0.5 : -0.5),
                Center.z() + Extent.z() * ((i & 4) ? 0.5 : -0.5)
        );
    }

    std::array<GA_Offset, 8> pt_off{};
    for (int i = 0; i < 8; i++)
    {
        pt_off[i] = gdp->appendPointOffset();
        gdp->setPos3(pt_off[i], vertices[i]);

        GA_RWHandleV3 gdp_handle_cd(gdp->addFloatTuple(GA_ATTRIB_POINT, "Cd", 3));
        gdp_handle_cd.set(pt_off[i], color);
    }

    static const int edges[12][2] = {
            {0, 1},
            {0, 4},
            {1, 3},
            {1, 5},
            {2, 0},
            {2, 3},
            {2, 6},
            {3, 7},
            {4, 5},
            {4, 6},
            {5, 7},
            {6, 7},
    };

    for (int i = 0; i < 12; i++)
    {
        GEO_PrimPoly *line = GEO_PrimPoly::build(gdp, 2, GU_POLY_OPEN);
        for (int j = 0; j < 2; j++)
            line->setVertexPoint(j, pt_off[edges[i][j]]);
    }
}

void SIM_FluidParticle::check_data() const {
    // Check Particle data
    if (!pbfParticle)
    {
        std::cout << "SIM_FluidParticle::check_ptr: pbfParticle is nullptr" << std::endl;
    }

    // show particle size
    /*std::cout << "pbfParticle->positions.size() = " << pbfParticle->positions.size() << std::endl;
    std::cout << "pbfParticle->velocities.size() = " << pbfParticle->velocities.size() << std::endl;
    std::cout << "pbfParticle->forces.size() = " << pbfParticle->forces.size() << std::endl;
    std::cout << "pbfParticle->densities.size() = " << pbfParticle->densities.size() << std::endl;
    std::cout << "pbfParticle->pressures.size() = " << pbfParticle->pressures.size() << std::endl;*/
    std::cout << "pbfParticle->point_neighbors.size() = " << pbfParticle->point_neighbors.size() << std::endl;
    /*std::cout << "pbfParticle->triangle_neighbors.size() = " << pbfParticle->triangle_neighbors.size() << std::endl;

    // show particle positions
    for (const auto &pair: pbfParticle->positions)
    {
        std::cout << "pbfParticle->positions[" << pair.first << "] = " << pair.second << std::endl;
    }

    // show particle velocities
    for (const auto &pair: pbfParticle->velocities)
    {
        std::cout << "pbfParticle->velocities[" << pair.first << "] = " << pair.second << std::endl;
    }

    // show particle forces
    for (const auto &pair: pbfParticle->forces)
    {
        std::cout << "pbfParticle->forces[" << pair.first << "] = " << pair.second << std::endl;
    }

    // show particle densities
    for (const auto &pair: pbfParticle->densities)
    {
        std::cout << "pbfParticle->densities[" << pair.first << "] = " << pair.second << std::endl;
    }

    // show particle pressures
    for (const auto &pair: pbfParticle->pressures)
    {
        std::cout << "pbfParticle->pressures[" << pair.first << "] = " << pair.second << std::endl;
    }*/

    // show particle point neighbors
    for (const auto &pair: pbfParticle->point_neighbors)
    {
        std::cout << "pbfParticle->point_neighbors[" << pair.first << "] = " << std::endl;
        for (const auto &neighbor: pair.second)
        {
            std::cout << "neighbor.pt_off = " << neighbor.pt_off << std::endl;
            std::cout << "neighbor.pt_pos = " << neighbor.pt_pos << std::endl;
        }
    }

    /*// show particle triangle neighbors
    for (const auto &pair: pbfParticle->triangle_neighbors)
    {
        std::cout << "pbfParticle->triangle_neighbors[" << pair.first << "] = " << std::endl;
        for (const auto &neighbor: pair.second)
        {
            std::cout << "neighbor.prim_off = " << neighbor.prim_off << std::endl;
            std::cout << "neighbor.pt1.pt_off = " << neighbor.pt1.pt_off << std::endl;
            std::cout << "neighbor.pt1.pt_pos = " << neighbor.pt1.pt_pos << std::endl;
            std::cout << "neighbor.pt2.pt_off = " << neighbor.pt2.pt_off << std::endl;
            std::cout << "neighbor.pt2.pt_pos = " << neighbor.pt2.pt_pos << std::endl;
            std::cout << "neighbor.pt3.pt_off = " << neighbor.pt3.pt_off << std::endl;
            std::cout << "neighbor.pt3.pt_pos = " << neighbor.pt3.pt_pos << std::endl;
        }
    }*/
}







