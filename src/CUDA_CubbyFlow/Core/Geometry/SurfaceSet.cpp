// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Geometry/SurfaceSet.hpp>

#include <limits>

namespace CubbyFlow
{
template <size_t N>
SurfaceSet<N>::SurfaceSet(
    const ConstArrayView1<std::shared_ptr<Surface<N>>>& others,
    const Transform<N>& _transform, bool _isNormalFlipped)
    : Surface<N>(_transform, _isNormalFlipped), m_surfaces(others)
{
    for (const auto& surface : m_surfaces)
    {
        if (!surface->IsBounded())
        {
            m_unboundedSurfaces.Append(surface);
        }
    }

    InvalidateBVH();
}

template <size_t N>
SurfaceSet<N>::SurfaceSet(const SurfaceSet& other)
    : Surface<N>(other),
      m_surfaces(other.m_surfaces),
      m_unboundedSurfaces(other.m_unboundedSurfaces)
{
    InvalidateBVH();
}

template <size_t N>
SurfaceSet<N>::SurfaceSet(SurfaceSet&& other) noexcept
    : Surface<N>(other),
      m_surfaces(std::move(other.m_surfaces)),
      m_unboundedSurfaces(std::move(other.m_unboundedSurfaces))
{
    InvalidateBVH();
}

template <size_t N>
SurfaceSet<N>& SurfaceSet<N>::operator=(const SurfaceSet& other)
{
    m_surfaces = other.m_surfaces;
    m_unboundedSurfaces = other.m_unboundedSurfaces;

    InvalidateBVH();

    return *this;
}

template <size_t N>
SurfaceSet<N>& SurfaceSet<N>::operator=(SurfaceSet&& other) noexcept
{
    m_surfaces = std::move(other.m_surfaces);
    m_unboundedSurfaces = std::move(other.m_unboundedSurfaces);

    InvalidateBVH();

    return *this;
}

template <size_t N>
void SurfaceSet<N>::UpdateQueryEngine()
{
    InvalidateBVH();
    BuildBVH();
}

template <size_t N>
bool SurfaceSet<N>::IsBounded() const
{
    // All surfaces should be bounded
    for (const auto& surface : m_surfaces)
    {
        if (!surface->IsBounded())
        {
            return false;
        }
    }

    // Empty set is not bounded
    return !m_surfaces.IsEmpty();
}

template <size_t N>
bool SurfaceSet<N>::IsValidGeometry() const
{
    // All surfaces should be valid.
    for (const auto& surface : m_surfaces)
    {
        if (!surface->IsValidGeometry())
        {
            return false;
        }
    }

    // Empty set is not valid.
    return !m_surfaces.IsEmpty();
}

template <size_t N>
size_t SurfaceSet<N>::NumberOfSurfaces() const
{
    return m_surfaces.Length();
}

template <size_t N>
const std::shared_ptr<Surface<N>>& SurfaceSet<N>::SurfaceAt(size_t i) const
{
    return m_surfaces[i];
}

template <size_t N>
void SurfaceSet<N>::AddSurface(const std::shared_ptr<Surface<N>>& surface)
{
    m_surfaces.Append(surface);

    if (!surface->IsBounded())
    {
        m_unboundedSurfaces.Append(surface);
    }

    InvalidateBVH();
}

template <size_t N>
Vector<double, N> SurfaceSet<N>::ClosestPointLocal(
    const Vector<double, N>& otherPoint) const
{
    BuildBVH();

    const auto distanceFunc = [](const std::shared_ptr<Surface<N>>& surface,
                                 const Vector<double, N>& pt) {
        return surface->ClosestDistance(pt);
    };

    Vector<double, N> result =
        Vector<double, N>::MakeConstant(std::numeric_limits<double>::max());
    result.Fill(std::numeric_limits<double>::max());

    const auto queryResult = m_bvh.Nearest(otherPoint, distanceFunc);
    if (queryResult.item != nullptr)
    {
        result = (*queryResult.item)->ClosestPoint(otherPoint);
    }

    double minDist = queryResult.distance;

    for (const auto& surface : m_unboundedSurfaces)
    {
        Vector<double, N> pt = surface->ClosestPoint(otherPoint);

        if (const double dist = pt.DistanceTo(otherPoint); dist < minDist)
        {
            minDist = dist;
            result = surface->ClosestPoint(otherPoint);
        }
    }

    return result;
}

template <size_t N>
BoundingBox<double, N> SurfaceSet<N>::BoundingBoxLocal() const
{
    BuildBVH();

    return m_bvh.GetBoundingBox();
}

template <size_t N>
SurfaceRayIntersection<N> SurfaceSet<N>::ClosestIntersectionLocal(
    const Ray<double, N>& ray) const
{
    BuildBVH();

    const auto testFunc = [](const std::shared_ptr<Surface<N>>& surface,
                             const Ray<double, N>& ray) {
        SurfaceRayIntersection<N> result = surface->ClosestIntersection(ray);
        return result.distance;
    };

    const auto queryResult = m_bvh.ClosestIntersection(ray, testFunc);

    SurfaceRayIntersection<N> result;
    result.distance = queryResult.distance;
    result.isIntersecting = queryResult.item != nullptr;

    if (queryResult.item != nullptr)
    {
        result.point = ray.PointAt(queryResult.distance);
        result.normal = (*queryResult.item)->ClosestNormal(result.point);
    }

    for (const auto& surface : m_unboundedSurfaces)
    {
        if (SurfaceRayIntersection<N> localResult =
                surface->ClosestIntersection(ray);
            localResult.distance < result.distance)
        {
            result = localResult;
        }
    }

    return result;
}

template <size_t N>
Vector<double, N> SurfaceSet<N>::ClosestNormalLocal(
    const Vector<double, N>& otherPoint) const
{
    BuildBVH();

    const auto distanceFunc = [](const std::shared_ptr<Surface<N>>& surface,
                                 const Vector<double, N>& pt) {
        return surface->ClosestDistance(pt);
    };

    Vector<double, N> result = Vector<double, N>::MakeUnitX();
    const auto queryResult = m_bvh.Nearest(otherPoint, distanceFunc);

    if (queryResult.item != nullptr)
    {
        result = (*queryResult.item)->ClosestNormal(otherPoint);
    }

    double minDist = queryResult.distance;

    for (const auto& surface : m_unboundedSurfaces)
    {
        Vector<double, N> pt = surface->ClosestPoint(otherPoint);

        if (const double dist = pt.DistanceTo(otherPoint); dist < minDist)
        {
            minDist = dist;
            result = surface->ClosestNormal(otherPoint);
        }
    }

    return result;
}

template <size_t N>
bool SurfaceSet<N>::IntersectsLocal(const Ray<double, N>& ray) const
{
    BuildBVH();

    const auto testFunc = [](const std::shared_ptr<Surface<N>>& surface,
                             const Ray<double, N>& ray) {
        return surface->Intersects(ray);
    };

    bool result = m_bvh.Intersects(ray, testFunc);

    for (const auto& surface : m_unboundedSurfaces)
    {
        result |= surface->Intersects(ray);
    }

    return result;
}

template <size_t N>
double SurfaceSet<N>::ClosestDistanceLocal(
    const Vector<double, N>& otherPoint) const
{
    BuildBVH();

    const auto distanceFunc = [](const std::shared_ptr<Surface<N>>& surface,
                                 const Vector<double, N>& pt) {
        return surface->ClosestDistance(pt);
    };

    const auto queryResult = m_bvh.Nearest(otherPoint, distanceFunc);
    double minDist = queryResult.distance;

    for (const auto& surface : m_unboundedSurfaces)
    {
        Vector<double, N> pt = surface->ClosestPoint(otherPoint);

        if (const double dist = pt.DistanceTo(otherPoint); dist < minDist)
        {
            minDist = dist;
        }
    }

    return minDist;
}

template <size_t N>
bool SurfaceSet<N>::IsInsideLocal(const Vector<double, N>& otherPoint) const
{
    return std::any_of(m_surfaces.begin(), m_surfaces.end(),
                       [&](const std::shared_ptr<Surface<N>> surface) {
                           return surface->IsInside(otherPoint);
                       });
}

template <size_t N>
void SurfaceSet<N>::InvalidateBVH() const
{
    m_bvhInvalidated = true;
}

template <size_t N>
void SurfaceSet<N>::BuildBVH() const
{
    if (m_bvhInvalidated)
    {
        Array1<std::shared_ptr<Surface<N>>> surfs;
        Array1<BoundingBox<double, N>> bounds;

        for (const auto& surface : m_surfaces)
        {
            if (surface->IsBounded())
            {
                surfs.Append(surface);
                bounds.Append(surface->GetBoundingBox());
            }
        }

        m_bvh.Build(surfs, bounds);
        m_bvhInvalidated = false;
    }
}

template <size_t N>
typename SurfaceSet<N>::Builder SurfaceSet<N>::GetBuilder()
{
    return Builder{};
}

template <size_t N>
typename SurfaceSet<N>::Builder& SurfaceSet<N>::Builder::WithSurfaces(
    const ConstArrayView1<std::shared_ptr<Surface<N>>>& others)
{
    m_surfaces = others;
    return *this;
}

template <size_t N>
SurfaceSet<N> SurfaceSet<N>::Builder::Build() const
{
    return SurfaceSet{ m_surfaces, m_transform, m_isNormalFlipped };
}

template <size_t N>
std::shared_ptr<SurfaceSet<N>> SurfaceSet<N>::Builder::MakeShared() const
{
    return std::shared_ptr<SurfaceSet>{ new SurfaceSet(m_surfaces, m_transform,
                                                       m_isNormalFlipped),
                                        [](SurfaceSet* obj) { delete obj; } };
}

template class SurfaceSet<2>;

template class SurfaceSet<3>;
}  // namespace CubbyFlow