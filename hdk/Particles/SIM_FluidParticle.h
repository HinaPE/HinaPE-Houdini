//
// Created by LiYifan on 2024/2/26.
//

#ifndef HINAPE_HOUDINI_SIM_FLUIDPARTICLE_H
#define HINAPE_HOUDINI_SIM_FLUIDPARTICLE_H

#include <SIM/SIM_Data.h>
#include <SIM/SIM_DataUtils.h>
#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Utils.h>
#include <UT/UT_WorkBuffer.h>
#include <SIM/SIM_GeometryCopy.h>

#include "PbfParticle.h"
#include "Particles.h"

class SIM_FluidParticle : public SIM_Data, public SIM_OptionsUser
{
public:
    static const char *DATANAME;

    bool Configured = false;
    PbfParticlePtr pbfParticle;

    std::map<GA_Offset, GA_Size> offset2index;
    std::map<GA_Size, GA_Offset> index2offset;

    GETSET_DATA_FUNCS_V3("FluidDomain", FluidDomain)
    GETSET_DATA_FUNCS_F("TargetDensity", TargetDensity)
    GETSET_DATA_FUNCS_F("TargetSpacing", TargetSpacing)
    GETSET_DATA_FUNCS_F("KernelRadiusOverTargetSpacing", KernelRadiusOverTargetSpacing)
    GETSET_DATA_FUNCS_V3("Gravity", Gravity)

    GET_GUIDE_FUNC_B(SIM_NAME_SHOWGUIDE, ShowGuideGeometry, true);
    GET_GUIDE_FUNC_V3("DomainColor", DomainColor, (.0156356, 0, .5))

    //////////////////
    void load(SIM_GeometryCopy *geo);
    void commit(SIM_GeometryCopy *geo) const;
    void force_keep_boundary();
    static void setup_gdp(GU_Detail *gdp) ;
    void for_each_offset(const std::function<void(const GA_Offset &)> &func);
    void for_each_neighbor_self(const GA_Offset &pt_off, const std::function<void(const GA_Offset &, const UT_Vector3 &)> &func) const;
    void check_data() const;

protected:
    explicit SIM_FluidParticle(const SIM_DataFactory *factory) : SIM_Data(factory), SIM_OptionsUser(this) {}
    ~SIM_FluidParticle() override = default;
    void initializeSubclass() override;
    void makeEqualSubclass(const SIM_Data *source) override;
    static const SIM_DopDescription *GetDescription();

    SIM_Guide *createGuideObjectSubclass() const override;
    void buildGuideGeometrySubclass(const SIM_RootData &root, const SIM_Options &options, const GU_DetailHandle &gdh, UT_DMatrix4 *xform, const SIM_Time &t) const override;

    DECLARE_STANDARD_GETCASTTOTYPE();
    DECLARE_DATAFACTORY(SIM_FluidParticle, SIM_Data, "Fluid_Particle", GetDescription());
};

#endif //HINAPE_HOUDINI_SIM_FLUIDPARTICLE_H
