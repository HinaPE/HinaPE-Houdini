//
// Created by LiYifan on 2024/2/29.
//

#ifndef HINAPE_HOUDINI_ALIGNEDBOX_H
#define HINAPE_HOUDINI_ALIGNEDBOX_H

#include <iostream>
#include <vector>
#include "Particles/Particles.h"

class AlignedBox
{
public:
    AlignedBox() : v0(-1, -1, -1), v1(1, 1, 1) {}
    AlignedBox(const UT_Vector3& v0, const UT_Vector3& v1) : v0(v0), v1(v1) {}
    AlignedBox(const AlignedBox& other) : v0(other.v0), v1(other.v1) {}
    bool intersect(const AlignedBox &abox, AlignedBox &bbox) const{
        for (int i = 0; i < 3; i++)
        {
            if (v1.x() <= abox.v0.x() || v0.x() >= abox.v1.x())
            {
                return false;
            }
        }
        UT_Vector3 res1 = UT_Vector3(0, 0, 0);
        UT_Vector3 res2 = UT_Vector3(0, 0, 0);

        res1.x() = v0.x() > abox.v0.x() ? v0.x() : abox.v0.x();
        res1.y() = v0.y() > abox.v0.y() ? v0.y() : abox.v0.y();
        res1.z() = v0.z() > abox.v0.z() ? v0.z() : abox.v0.z();

        bbox.v0 = res1;

        res2.x() = v1.x() < abox.v1.x() ? v1.x() : abox.v1.x();
        res2.y() = v1.y() < abox.v1.y() ? v1.y() : abox.v1.y();
        res2.z() = v1.z() < abox.v1.z() ? v1.z() : abox.v1.z();

        bbox.v1 = res2;

        for (int i = 0; i < 3; i++)
        {
            if (v1.x() <= abox.v1.x())
            {
                bbox.v1.x() = v1.x();
            }
            else
            {
                bbox.v1.x() = abox.v1.x();
            }

            if (v0.x() <= abox.v0.x())
            {
                bbox.v0.x() = abox.v0.x();
            }
            else
            {
                bbox.v0.x() = v0.x();
            }
        }

        return true;
    }
    ~AlignedBox() = default;

    UT_Vector3 v0;
    UT_Vector3 v1;
};

#endif //HINAPE_HOUDINI_ALIGNEDBOX_H
