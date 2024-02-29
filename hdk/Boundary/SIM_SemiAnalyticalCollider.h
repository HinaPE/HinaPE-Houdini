//
// Created by LiYifan on 2024/2/29.
//

#ifndef HINAPE_HOUDINI_SIM_SEMIANALYTICALCOLLIDER_H
#define HINAPE_HOUDINI_SIM_SEMIANALYTICALCOLLIDER_H

#include <SIM/SIM_Collider.h>
#include <PRM/PRM_Default.h>
#include <SIM/SIM_GeometryCopy.h>

#include "SemiAnalyticalBoundary.h"

class SIM_SemiAnalyticalCollider : public SIM_Collider
{
public:
    static const char *DATANAME;
    SemiAnalyticalBoundaryPtr boundary;
    bool Configured = false;

    //////////
    void check_data();
    void check_AABB();
    void for_each_triangle(const std::function<void(const GA_Offset &, const std::vector<size_t> &)> &func);

protected:
    SIM_SemiAnalyticalCollider(const SIM_DataFactory *factory) : SIM_Collider(factory) {}
    ~SIM_SemiAnalyticalCollider() override = default;
    void initializeSubclass() override;
    void makeEqualSubclass(const SIM_Data *source) override;
    static const SIM_DopDescription *getDopDescription();

DECLARE_STANDARD_GETCASTTOTYPE();
DECLARE_DATAFACTORY(SIM_SemiAnalyticalCollider,
                    SIM_Collider,
                    "SemiAnalyticalCollider",
                    getDopDescription());
};

#endif //HINAPE_HOUDINI_SIM_SEMIANALYTICALCOLLIDER_H
