//
// Created by LiYifan on 2024/2/29.
//

#ifndef HINAPE_HOUDINI_SEMIANALYTICALBOUNDARY_H
#define HINAPE_HOUDINI_SEMIANALYTICALBOUNDARY_H

#include <iostream>
#include <map>
#include <vector>
#include <UT/UT_Vector3.h>
#include <GA/GA_Types.h>

#include "AlignedBox.h"

class SemiAnalyticalBoundary
{
public:
    SemiAnalyticalBoundary() = default;
    ~SemiAnalyticalBoundary() = default;

public:
    std::map<GA_Offset,UT_Vector3> vertices;
    std::map<GA_Offset,std::vector<size_t>> faces;

    std::map<GA_Offset,AlignedBox> mQueriedAABB;
};

using SemiAnalyticalBoundaryPtr = std::shared_ptr<SemiAnalyticalBoundary>;

#endif //HINAPE_HOUDINI_SEMIANALYTICALBOUNDARY_H
