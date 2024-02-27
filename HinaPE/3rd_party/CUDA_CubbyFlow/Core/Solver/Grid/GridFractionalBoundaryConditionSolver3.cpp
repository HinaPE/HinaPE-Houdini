// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Array/ArrayUtils.hpp>
#include <Core/Geometry/ImplicitSurface.hpp>
#include <Core/Geometry/SurfaceToImplicit.hpp>
#include <Core/Solver/Grid/GridFractionalBoundaryConditionSolver3.hpp>
#include <Core/Utils/LevelSetUtils.hpp>
#include <Core/Utils/PhysicsHelpers.hpp>

namespace CubbyFlow
{
void GridFractionalBoundaryConditionSolver3::ConstrainVelocity(
    FaceCenteredGrid3* velocity, unsigned int extrapolationDepth)
{
    Vector3UZ size = velocity->Resolution();

    if (m_colliderSDF == nullptr || m_colliderSDF->Resolution() != size)
    {
        UpdateCollider(GetCollider(), size, velocity->GridSpacing(),
                       velocity->Origin());
    }

    ArrayView3<double> u = velocity->UView();
    ArrayView3<double> v = velocity->VView();
    ArrayView3<double> w = velocity->WView();
    auto uPos = velocity->UPosition();
    auto vPos = velocity->VPosition();
    auto wPos = velocity->WPosition();

    Array3<double> uTemp{ u.Size() };
    Array3<double> vTemp{ v.Size() };
    Array3<double> wTemp{ w.Size() };
    Array3<char> uMarker{ u.Size(), 1 };
    Array3<char> vMarker{ v.Size(), 1 };
    Array3<char> wMarker{ w.Size(), 1 };

    Vector3D h = velocity->GridSpacing();

    // Assign collider's velocity first and initialize markers
    velocity->ParallelForEachUIndex([&](const Vector3UZ& idx) {
        const Vector3D pt = uPos(idx);
        const double phi0 =
            m_colliderSDF->Sample(pt - Vector3D{ 0.5 * h.x, 0.0, 0.0 });
        const double phi1 =
            m_colliderSDF->Sample(pt + Vector3D{ 0.5 * h.x, 0.0, 0.0 });
        double frac = FractionInsideSDF(phi0, phi1);
        frac = 1.0 - std::clamp(frac, 0.0, 1.0);

        if (frac > 0.0)
        {
            uMarker(idx) = 1;
        }
        else
        {
            const Vector3D colliderVel = GetCollider()->VelocityAt(pt);
            u(idx) = colliderVel.x;
            uMarker(idx) = 0;
        }
    });

    velocity->ParallelForEachVIndex([&](const Vector3UZ& idx) {
        const Vector3D pt = vPos(idx);
        const double phi0 =
            m_colliderSDF->Sample(pt - Vector3D{ 0.0, 0.5 * h.y, 0.0 });
        const double phi1 =
            m_colliderSDF->Sample(pt + Vector3D{ 0.0, 0.5 * h.y, 0.0 });
        double frac = FractionInsideSDF(phi0, phi1);
        frac = 1.0 - std::clamp(frac, 0.0, 1.0);

        if (frac > 0.0)
        {
            vMarker(idx) = 1;
        }
        else
        {
            const Vector3D colliderVel = GetCollider()->VelocityAt(pt);
            v(idx) = colliderVel.y;
            vMarker(idx) = 0;
        }
    });

    velocity->ParallelForEachWIndex([&](const Vector3UZ& idx) {
        const Vector3D pt = wPos(idx);
        const double phi0 =
            m_colliderSDF->Sample(pt - Vector3D{ 0.0, 0.0, 0.5 * h.z });
        const double phi1 =
            m_colliderSDF->Sample(pt + Vector3D{ 0.0, 0.0, 0.5 * h.z });
        double frac = FractionInsideSDF(phi0, phi1);
        frac = 1.0 - std::clamp(frac, 0.0, 1.0);

        if (frac > 0.0)
        {
            wMarker(idx) = 1;
        }
        else
        {
            const Vector3D colliderVel = GetCollider()->VelocityAt(pt);
            w(idx) = colliderVel.z;
            wMarker(idx) = 0;
        }
    });

    // Free-slip: Extrapolate fluid velocity into the collider
    ExtrapolateToRegion(velocity->UView(), uMarker, extrapolationDepth, u);
    ExtrapolateToRegion(velocity->VView(), vMarker, extrapolationDepth, v);
    ExtrapolateToRegion(velocity->WView(), wMarker, extrapolationDepth, w);

    // No-flux: project the extrapolated velocity to the collider's surface
    // normal
    velocity->ParallelForEachUIndex([&](const Vector3UZ& idx) {
        const Vector3D pt = uPos(idx);

        if (IsInsideSDF(m_colliderSDF->Sample(pt)))
        {
            const Vector3D colliderVel = GetCollider()->VelocityAt(pt);
            const Vector3D vel = velocity->Sample(pt);
            const Vector3D g = m_colliderSDF->Gradient(pt);

            if (g.LengthSquared() > 0.0)
            {
                const Vector3D n = g.Normalized();
                const Vector3D velr = vel - colliderVel;
                const Vector3D velt = ProjectAndApplyFriction(
                    velr, n, GetCollider()->GetFrictionCoefficient());
                const Vector3D velp = velt + colliderVel;

                uTemp(idx) = velp.x;
            }
            else
            {
                uTemp(idx) = colliderVel.x;
            }
        }
        else
        {
            uTemp(idx) = u(idx);
        }
    });

    velocity->ParallelForEachVIndex([&](const Vector3UZ& idx) {
        const Vector3D pt = vPos(idx);

        if (IsInsideSDF(m_colliderSDF->Sample(pt)))
        {
            const Vector3D colliderVel = GetCollider()->VelocityAt(pt);
            const Vector3D vel = velocity->Sample(pt);
            const Vector3D g = m_colliderSDF->Gradient(pt);

            if (g.LengthSquared() > 0.0)
            {
                const Vector3D n = g.Normalized();
                const Vector3D velr = vel - colliderVel;
                const Vector3D velt = ProjectAndApplyFriction(
                    velr, n, GetCollider()->GetFrictionCoefficient());
                const Vector3D velp = velt + colliderVel;

                vTemp(idx) = velp.y;
            }
            else
            {
                vTemp(idx) = colliderVel.y;
            }
        }
        else
        {
            vTemp(idx) = v(idx);
        }
    });

    velocity->ParallelForEachWIndex([&](const Vector3UZ& idx) {
        const Vector3D pt = wPos(idx);

        if (IsInsideSDF(m_colliderSDF->Sample(pt)))
        {
            const Vector3D colliderVel = GetCollider()->VelocityAt(pt);
            const Vector3D vel = velocity->Sample(pt);
            const Vector3D g = m_colliderSDF->Gradient(pt);

            if (g.LengthSquared() > 0.0)
            {
                const Vector3D n = g.Normalized();
                const Vector3D velr = vel - colliderVel;
                const Vector3D velt = ProjectAndApplyFriction(
                    velr, n, GetCollider()->GetFrictionCoefficient());
                const Vector3D velp = velt + colliderVel;

                wTemp(idx) = velp.z;
            }
            else
            {
                wTemp(idx) = colliderVel.z;
            }
        }
        else
        {
            wTemp(idx) = w(idx);
        }
    });

    // Transfer results
    ParallelForEachIndex(u.Size(), [&](size_t i, size_t j, size_t k) {
        u(i, j, k) = uTemp(i, j, k);
    });
    ParallelForEachIndex(v.Size(), [&](size_t i, size_t j, size_t k) {
        v(i, j, k) = vTemp(i, j, k);
    });
    ParallelForEachIndex(w.Size(), [&](size_t i, size_t j, size_t k) {
        w(i, j, k) = wTemp(i, j, k);
    });

    // No-flux: Project velocity on the domain boundary if closed
    if (GetClosedDomainBoundaryFlag() & DIRECTION_LEFT)
    {
        for (size_t k = 0; k < u.Size().z; ++k)
        {
            for (size_t j = 0; j < u.Size().y; ++j)
            {
                u(0, j, k) = 0;
            }
        }
    }
    if (GetClosedDomainBoundaryFlag() & DIRECTION_RIGHT)
    {
        for (size_t k = 0; k < u.Size().z; ++k)
        {
            for (size_t j = 0; j < u.Size().y; ++j)
            {
                u(u.Size().x - 1, j, k) = 0;
            }
        }
    }
    if (GetClosedDomainBoundaryFlag() & DIRECTION_DOWN)
    {
        for (size_t k = 0; k < v.Size().z; ++k)
        {
            for (size_t i = 0; i < v.Size().x; ++i)
            {
                v(i, 0, k) = 0;
            }
        }
    }
    if (GetClosedDomainBoundaryFlag() & DIRECTION_UP)
    {
        for (size_t k = 0; k < v.Size().z; ++k)
        {
            for (size_t i = 0; i < v.Size().x; ++i)
            {
                v(i, v.Size().y - 1, k) = 0;
            }
        }
    }
    if (GetClosedDomainBoundaryFlag() & DIRECTION_BACK)
    {
        for (size_t j = 0; j < w.Size().y; ++j)
        {
            for (size_t i = 0; i < w.Size().x; ++i)
            {
                w(i, j, 0) = 0;
            }
        }
    }
    if (GetClosedDomainBoundaryFlag() & DIRECTION_FRONT)
    {
        for (size_t j = 0; j < w.Size().y; ++j)
        {
            for (size_t i = 0; i < w.Size().x; ++i)
            {
                w(i, j, w.Size().z - 1) = 0;
            }
        }
    }
}

ScalarField3Ptr GridFractionalBoundaryConditionSolver3::GetColliderSDF() const
{
    return m_colliderSDF;
}

VectorField3Ptr
GridFractionalBoundaryConditionSolver3::GetColliderVelocityField() const
{
    return m_colliderVel;
}

void GridFractionalBoundaryConditionSolver3::OnColliderUpdated(
    const Vector3UZ& gridSize, const Vector3D& gridSpacing,
    const Vector3D& gridOrigin)
{
    if (m_colliderSDF == nullptr)
    {
        m_colliderSDF = std::make_shared<CellCenteredScalarGrid3>();
    }

    m_colliderSDF->Resize(gridSize, gridSpacing, gridOrigin);

    if (GetCollider() != nullptr)
    {
        Surface3Ptr surface = GetCollider()->GetSurface();
        ImplicitSurface3Ptr implicitSurface =
            std::dynamic_pointer_cast<ImplicitSurface3>(surface);
        if (implicitSurface == nullptr)
        {
            implicitSurface = std::make_shared<SurfaceToImplicit3>(surface);
        }

        m_colliderSDF->Fill([&](const Vector3D& pt) {
            return implicitSurface->SignedDistance(pt);
        });

        m_colliderVel = CustomVectorField3::Builder{}
                            .WithFunction([&](const Vector3D& x) {
                                return GetCollider()->VelocityAt(x);
                            })
                            .WithDerivativeResolution(gridSpacing.x)
                            .MakeShared();
    }
    else
    {
        m_colliderSDF->Fill(std::numeric_limits<double>::max());

        m_colliderVel =
            CustomVectorField3::Builder{}
                .WithFunction([](const Vector3D&) { return Vector3D{}; })
                .WithDerivativeResolution(gridSpacing.x)
                .MakeShared();
    }
}
}  // namespace CubbyFlow