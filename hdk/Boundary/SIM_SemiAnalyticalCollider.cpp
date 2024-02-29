//
// Created by LiYifan on 2024/2/29.
//

#include "SIM_SemiAnalyticalCollider.h"

#include <SIM/SIM_Engine.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_GuideShared.h>
#include <SIM/SIM_ColliderLabel.h>
#include <SIM/SIM_ForceGravity.h>
#include <SIM/SIM_Time.h>
#include <SIM/SIM_Utils.h>

#include <PRM/PRM_Name.h>
#include <PRM/PRM_Template.h>
#include <PRM/PRM_Shared.h>
#include <PRM/PRM_Default.h>
#include <PRM/PRM_Utils.h>
#include <PRM/PRM_SpareData.h>

#include <UT/UT_WorkBuffer.h>
#include <UT/UT_NetMessage.h>

void SIM_SemiAnalyticalCollider::initializeSubclass() {
    SIM_Data::initializeSubclass();
    Configured = false;
    boundary = nullptr;
}

void SIM_SemiAnalyticalCollider::makeEqualSubclass(const SIM_Data *source) {
    SIM_Data::makeEqualSubclass(source);
    const SIM_SemiAnalyticalCollider *src = SIM_DATA_CASTCONST(source, SIM_SemiAnalyticalCollider);

    this->Configured = src->Configured;
    this->boundary = src->boundary;
}

const char *SIM_SemiAnalyticalCollider::DATANAME = "SemiAnalyticalCollider";
const SIM_DopDescription *SIM_SemiAnalyticalCollider::getDopDescription() {
    static std::array<PRM_Template, 1> PRMS{
            PRM_Template()
    };

    static SIM_DopDescription DESC(true,
                                   "semi_analytical_collider",
                                   "Semi Analytical Collider",
                                   DATANAME,
                                   classname(),
                                   PRMS.data());
    return &DESC;
}

void SIM_SemiAnalyticalCollider::check_data() {
    if(!boundary)
    {
        std::cout << "Boundary is nullptr" << std::endl;
    }

    // show size
    std::cout << "Boundary PointSet Size: " << boundary->vertices.size() << std::endl;
    std::cout << "Boundary TriangleSet Size: " << boundary->faces.size() << std::endl;

    // show first 10 vertices
    std::cout << "First 10 Vertices: " << std::endl;
    for(int i = 0; i < 10; i++)
    {
        std::cout << "Vertices " << i << ": " << boundary->vertices[i] << std::endl;
    }

    // show first 10 faces
    std::cout << "First 10 Faces: " << std::endl;
    for(int i = 0; i < 10; i++)
    {
        std::cout << "Face " << i << ": ";
        for(const auto& index : boundary->faces[i])
        {
            std::cout << index << " ";
        }
        std::cout << std::endl;
    }
}


