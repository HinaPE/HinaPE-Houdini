// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/FDM/FDMUtils.hpp>
#include <Core/Solver/LevelSet/FMMLevelSetSolver3.hpp>
#include <Core/Utils/LevelSetUtils.hpp>

#include <queue>

namespace CubbyFlow
{
static const char UNKNOWN = 0;
static const char KNOWN = 1;
static const char TRIAL = 2;

// Find geometric solution near the boundary
inline double SolveQuadNearBoundary(const Array3<char>& markers,
                                    ArrayView3<double> output,
                                    const Vector3D& gridSpacing,
                                    const Vector3D& invGridSpacingSqr,
                                    double sign, size_t i, size_t j, size_t k)
{
    UNUSED_VARIABLE(markers);
    UNUSED_VARIABLE(invGridSpacingSqr);

    const Vector3UZ size = output.Size();

    bool hasX = false;
    double phiX = std::numeric_limits<double>::max();

    if (i > 0)
    {
        if (IsInsideSDF(sign * output(i - 1, j, k)))
        {
            hasX = true;
            phiX = std::min(phiX, sign * output(i - 1, j, k));
        }
    }

    if (i + 1 < size.x)
    {
        if (IsInsideSDF(sign * output(i + 1, j, k)))
        {
            hasX = true;
            phiX = std::min(phiX, sign * output(i + 1, j, k));
        }
    }

    bool hasY = false;
    double phiY = std::numeric_limits<double>::max();

    if (j > 0)
    {
        if (IsInsideSDF(sign * output(i, j - 1, k)))
        {
            hasY = true;
            phiY = std::min(phiY, sign * output(i, j - 1, k));
        }
    }

    if (j + 1 < size.y)
    {
        if (IsInsideSDF(sign * output(i, j + 1, k)))
        {
            hasY = true;
            phiY = std::min(phiY, sign * output(i, j + 1, k));
        }
    }

    bool hasZ = false;
    double phiZ = std::numeric_limits<double>::max();

    if (k > 0)
    {
        if (IsInsideSDF(sign * output(i, j, k - 1)))
        {
            hasZ = true;
            phiZ = std::min(phiZ, sign * output(i, j, k - 1));
        }
    }

    if (k + 1 < size.z)
    {
        if (IsInsideSDF(sign * output(i, j, k + 1)))
        {
            hasZ = true;
            phiZ = std::min(phiZ, sign * output(i, j, k + 1));
        }
    }

    assert(hasX || hasY || hasZ);

    const double distToBndX = gridSpacing.x * std::abs(output(i, j, k)) /
                              (std::abs(output(i, j, k)) + std::abs(phiX));
    const double distToBndY = gridSpacing.y * std::abs(output(i, j, k)) /
                              (std::abs(output(i, j, k)) + std::abs(phiY));
    const double distToBndZ = gridSpacing.z * std::abs(output(i, j, k)) /
                              (std::abs(output(i, j, k)) + std::abs(phiZ));

    double denomSqr = 0.0;

    if (hasX)
    {
        denomSqr += 1.0 / Square(distToBndX);
    }
    if (hasY)
    {
        denomSqr += 1.0 / Square(distToBndY);
    }
    if (hasZ)
    {
        denomSqr += 1.0 / Square(distToBndZ);
    }

    const double solution = 1.0 / std::sqrt(denomSqr);

    return sign * solution;
}

inline double SolveQuad(const Array3<char>& markers, ArrayView3<double> output,
                        const Vector3D& gridSpacing,
                        const Vector3D& invGridSpacingSqr, size_t i, size_t j,
                        size_t k)
{
    const Vector3UZ size = output.Size();

    bool hasX = false;
    double phiX = std::numeric_limits<double>::max();

    if (i > 0)
    {
        if (markers(i - 1, j, k) == KNOWN)
        {
            hasX = true;
            phiX = std::min(phiX, output(i - 1, j, k));
        }
    }

    if (i + 1 < size.x)
    {
        if (markers(i + 1, j, k) == KNOWN)
        {
            hasX = true;
            phiX = std::min(phiX, output(i + 1, j, k));
        }
    }

    bool hasY = false;
    double phiY = std::numeric_limits<double>::max();

    if (j > 0)
    {
        if (markers(i, j - 1, k) == KNOWN)
        {
            hasY = true;
            phiY = std::min(phiY, output(i, j - 1, k));
        }
    }

    if (j + 1 < size.y)
    {
        if (markers(i, j + 1, k) == KNOWN)
        {
            hasY = true;
            phiY = std::min(phiY, output(i, j + 1, k));
        }
    }

    bool hasZ = false;
    double phiZ = std::numeric_limits<double>::max();

    if (k > 0)
    {
        if (markers(i, j, k - 1) == KNOWN)
        {
            hasZ = true;
            phiZ = std::min(phiZ, output(i, j, k - 1));
        }
    }

    if (k + 1 < size.z)
    {
        if (markers(i, j, k + 1) == KNOWN)
        {
            hasZ = true;
            phiZ = std::min(phiZ, output(i, j, k + 1));
        }
    }

    assert(hasX || hasY || hasZ);

    double solution = 0.0;

    // Initial guess
    if (hasX)
    {
        solution = std::max(solution, phiX + gridSpacing.x);
    }
    if (hasY)
    {
        solution = std::max(solution, phiY + gridSpacing.y);
    }
    if (hasZ)
    {
        solution = std::max(solution, phiZ + gridSpacing.z);
    }

    // Solve quad
    double a = 0.0;
    double b = 0.0;
    double c = -1.0;

    if (hasX)
    {
        a += invGridSpacingSqr.x;
        b -= phiX * invGridSpacingSqr.x;
        c += Square(phiX) * invGridSpacingSqr.x;
    }
    if (hasY)
    {
        a += invGridSpacingSqr.y;
        b -= phiY * invGridSpacingSqr.y;
        c += Square(phiY) * invGridSpacingSqr.y;
    }
    if (hasZ)
    {
        a += invGridSpacingSqr.z;
        b -= phiZ * invGridSpacingSqr.z;
        c += Square(phiZ) * invGridSpacingSqr.z;
    }

    const double det = b * b - a * c;

    if (det > 0.0)
    {
        solution = (-b + std::sqrt(det)) / a;
    }

    return solution;
}

void FMMLevelSetSolver3::Reinitialize(const ScalarGrid3& inputSDF,
                                      double maxDistance,
                                      ScalarGrid3* outputSDF)
{
    if (!inputSDF.HasSameShape(*outputSDF))
    {
        throw std::invalid_argument{
            "inputSDF and outputSDF have not same shape."
        };
    }

    Vector3UZ size = inputSDF.DataSize();
    Vector3D gridSpacing = inputSDF.GridSpacing();
    const Vector3D invGridSpacing = 1.0 / gridSpacing;
    Vector3D invGridSpacingSqr = ElemMul(invGridSpacing, invGridSpacing);
    Array3<char> markers{ size };

    auto output = outputSDF->DataView();

    ParallelForEachIndex(markers.Size(), [&](size_t i, size_t j, size_t k) {
        output(i, j, k) = inputSDF(i, j, k);
    });

    // Solve geometrically near the boundary
    ForEachIndex(markers.Size(), [&](size_t i, size_t j, size_t k) {
        if (!IsInsideSDF(output(i, j, k)) &&
            ((i > 0 && IsInsideSDF(output(i - 1, j, k))) ||
             (i + 1 < size.x && IsInsideSDF(output(i + 1, j, k))) ||
             (j > 0 && IsInsideSDF(output(i, j - 1, k))) ||
             (j + 1 < size.y && IsInsideSDF(output(i, j + 1, k))) ||
             (k > 0 && IsInsideSDF(output(i, j, k - 1))) ||
             (k + 1 < size.z && IsInsideSDF(output(i, j, k + 1)))))
        {
            output(i, j, k) = SolveQuadNearBoundary(
                markers, output, gridSpacing, invGridSpacingSqr, 1.0, i, j, k);
        }
        else if (IsInsideSDF(output(i, j, k)) &&
                 ((i > 0 && !IsInsideSDF(output(i - 1, j, k))) ||
                  (i + 1 < size.x && !IsInsideSDF(output(i + 1, j, k))) ||
                  (j > 0 && !IsInsideSDF(output(i, j - 1, k))) ||
                  (j + 1 < size.y && !IsInsideSDF(output(i, j + 1, k))) ||
                  (k > 0 && !IsInsideSDF(output(i, j, k - 1))) ||
                  (k + 1 < size.z && !IsInsideSDF(output(i, j, k + 1)))))
        {
            output(i, j, k) = SolveQuadNearBoundary(
                markers, output, gridSpacing, invGridSpacingSqr, -1.0, i, j, k);
        }
    });

    for (int sign = 0; sign < 2; ++sign)
    {
        // Build markers
        ParallelForEachIndex(markers.Size(), [&](size_t i, size_t j, size_t k) {
            if (IsInsideSDF(output(i, j, k)))
            {
                markers(i, j, k) = KNOWN;
            }
            else
            {
                markers(i, j, k) = UNKNOWN;
            }
        });

        auto compare = [&](const Vector3UZ& a, const Vector3UZ& b) {
            return output(a.x, a.y, a.z) > output(b.x, b.y, b.z);
        };

        // Enqueue initial candidates
        std::priority_queue<Vector3UZ, std::vector<Vector3UZ>,
                            decltype(compare)>
            trial(compare);
        ForEachIndex(markers.Size(), [&](size_t i, size_t j, size_t k) {
            if (markers(i, j, k) != KNOWN &&
                ((i > 0 && markers(i - 1, j, k) == KNOWN) ||
                 (i + 1 < size.x && markers(i + 1, j, k) == KNOWN) ||
                 (j > 0 && markers(i, j - 1, k) == KNOWN) ||
                 (j + 1 < size.y && markers(i, j + 1, k) == KNOWN) ||
                 (k > 0 && markers(i, j, k - 1) == KNOWN) ||
                 (k + 1 < size.z && markers(i, j, k + 1) == KNOWN)))
            {
                trial.push(Vector3UZ{ i, j, k });
                markers(i, j, k) = TRIAL;
            }
        });

        // Propagate
        while (!trial.empty())
        {
            const Vector3UZ idx = trial.top();
            trial.pop();

            const size_t i = idx.x;
            const size_t j = idx.y;
            const size_t k = idx.z;

            markers(i, j, k) = KNOWN;
            output(i, j, k) = SolveQuad(markers, output, gridSpacing,
                                        invGridSpacingSqr, i, j, k);

            if (output(i, j, k) > maxDistance)
            {
                break;
            }

            if (i > 0)
            {
                if (markers(i - 1, j, k) == UNKNOWN)
                {
                    markers(i - 1, j, k) = TRIAL;
                    output(i - 1, j, k) =
                        SolveQuad(markers, output, gridSpacing,
                                  invGridSpacingSqr, i - 1, j, k);
                    trial.push(Vector3UZ{ i - 1, j, k });
                }
            }

            if (i + 1 < size.x)
            {
                if (markers(i + 1, j, k) == UNKNOWN)
                {
                    markers(i + 1, j, k) = TRIAL;
                    output(i + 1, j, k) =
                        SolveQuad(markers, output, gridSpacing,
                                  invGridSpacingSqr, i + 1, j, k);
                    trial.push(Vector3UZ{ i + 1, j, k });
                }
            }

            if (j > 0)
            {
                if (markers(i, j - 1, k) == UNKNOWN)
                {
                    markers(i, j - 1, k) = TRIAL;
                    output(i, j - 1, k) =
                        SolveQuad(markers, output, gridSpacing,
                                  invGridSpacingSqr, i, j - 1, k);
                    trial.push(Vector3UZ{ i, j - 1, k });
                }
            }

            if (j + 1 < size.y)
            {
                if (markers(i, j + 1, k) == UNKNOWN)
                {
                    markers(i, j + 1, k) = TRIAL;
                    output(i, j + 1, k) =
                        SolveQuad(markers, output, gridSpacing,
                                  invGridSpacingSqr, i, j + 1, k);
                    trial.push(Vector3UZ{ i, j + 1, k });
                }
            }

            if (k > 0)
            {
                if (markers(i, j, k - 1) == UNKNOWN)
                {
                    markers(i, j, k - 1) = TRIAL;
                    output(i, j, k - 1) =
                        SolveQuad(markers, output, gridSpacing,
                                  invGridSpacingSqr, i, j, k - 1);
                    trial.push(Vector3UZ{ i, j, k - 1 });
                }
            }

            if (k + 1 < size.z)
            {
                if (markers(i, j, k + 1) == UNKNOWN)
                {
                    markers(i, j, k + 1) = TRIAL;
                    output(i, j, k + 1) =
                        SolveQuad(markers, output, gridSpacing,
                                  invGridSpacingSqr, i, j, k + 1);
                    trial.push(Vector3UZ{ i, j, k + 1 });
                }
            }
        }

        // Flip the sign
        ParallelForEachIndex(markers.Size(), [&](size_t i, size_t j, size_t k) {
            output(i, j, k) = -output(i, j, k);
        });
    }
}

void FMMLevelSetSolver3::Extrapolate(const ScalarGrid3& input,
                                     const ScalarField3& sdf,
                                     double maxDistance, ScalarGrid3* output)
{
    if (!input.HasSameShape(*output))
    {
        throw std::invalid_argument{ "input and output have not same shape." };
    }

    Array3<double> sdfGrid{ input.DataSize() };
    GridDataPositionFunc<3> pos = input.DataPosition();
    ParallelForEachIndex(sdfGrid.Size(), [&](size_t i, size_t j, size_t k) {
        sdfGrid(i, j, k) = sdf.Sample(pos(i, j, k));
    });

    Extrapolate(input.DataView(), sdfGrid.View(), input.GridSpacing(),
                maxDistance, output->DataView());
}

void FMMLevelSetSolver3::Extrapolate(const CollocatedVectorGrid3& input,
                                     const ScalarField3& sdf,
                                     double maxDistance,
                                     CollocatedVectorGrid3* output)
{
    if (!input.HasSameShape(*output))
    {
        throw std::invalid_argument{ "input and output have not same shape." };
    }

    Array3<double> sdfGrid{ input.DataSize() };
    GridDataPositionFunc<3> pos = input.DataPosition();
    ParallelForEachIndex(sdfGrid.Size(), [&](size_t i, size_t j, size_t k) {
        sdfGrid(i, j, k) = sdf.Sample(pos(i, j, k));
    });

    const Vector3D gridSpacing = input.GridSpacing();

    Array3<double> u{ input.DataSize() };
    Array3<double> u0{ input.DataSize() };
    Array3<double> v{ input.DataSize() };
    Array3<double> v0{ input.DataSize() };
    Array3<double> w{ input.DataSize() };
    Array3<double> w0{ input.DataSize() };

    input.ParallelForEachDataPointIndex([&](size_t i, size_t j, size_t k) {
        u(i, j, k) = input(i, j, k).x;
        v(i, j, k) = input(i, j, k).y;
        w(i, j, k) = input(i, j, k).z;
    });

    Extrapolate(u, sdfGrid.View(), gridSpacing, maxDistance, u0);
    Extrapolate(v, sdfGrid.View(), gridSpacing, maxDistance, v0);
    Extrapolate(w, sdfGrid.View(), gridSpacing, maxDistance, w0);

    output->ParallelForEachDataPointIndex([&](size_t i, size_t j, size_t k) {
        (*output)(i, j, k).x = u(i, j, k);
        (*output)(i, j, k).y = v(i, j, k);
        (*output)(i, j, k).z = w(i, j, k);
    });
}

void FMMLevelSetSolver3::Extrapolate(const FaceCenteredGrid3& input,
                                     const ScalarField3& sdf,
                                     double maxDistance,
                                     FaceCenteredGrid3* output)
{
    if (!input.HasSameShape(*output))
    {
        throw std::invalid_argument{
            "inputSDF and outputSDF have not same shape."
        };
    }

    const Vector3D& gridSpacing = input.GridSpacing();

    const ConstArrayView3<double> u = input.UView();
    auto uPos = input.UPosition();
    Array3<double> sdfAtU{ u.Size() };
    input.ParallelForEachUIndex(
        [&](const Vector3UZ& idx) { sdfAtU(idx) = sdf.Sample(uPos(idx)); });

    Extrapolate(u, sdfAtU, gridSpacing, maxDistance, output->UView());

    const ConstArrayView3<double> v = input.VView();
    auto vPos = input.VPosition();
    Array3<double> sdfAtV{ v.Size() };
    input.ParallelForEachVIndex(
        [&](const Vector3UZ& idx) { sdfAtV(idx) = sdf.Sample(vPos(idx)); });

    Extrapolate(v, sdfAtV, gridSpacing, maxDistance, output->VView());

    const ConstArrayView3<double> w = input.WView();
    auto wPos = input.WPosition();
    Array3<double> sdfAtW{ w.Size() };
    input.ParallelForEachWIndex(
        [&](const Vector3UZ& idx) { sdfAtW(idx) = sdf.Sample(wPos(idx)); });

    Extrapolate(w, sdfAtW, gridSpacing, maxDistance, output->WView());
}

void FMMLevelSetSolver3::Extrapolate(const ConstArrayView3<double>& input,
                                     const ConstArrayView3<double>& sdf,
                                     const Vector3D& gridSpacing,
                                     double maxDistance,
                                     ArrayView3<double> output)
{
    Vector3UZ size = input.Size();
    const Vector3D invGridSpacing = 1.0 / gridSpacing;

    // Build markers
    Array3<char> markers{ size, UNKNOWN };
    ParallelForEachIndex(markers.Size(), [&](size_t i, size_t j, size_t k) {
        if (IsInsideSDF(sdf(i, j, k)))
        {
            markers(i, j, k) = KNOWN;
        }
        output(i, j, k) = input(i, j, k);
    });

    auto compare = [&](const Vector3UZ& a, const Vector3UZ& b) {
        return sdf(a.x, a.y, a.z) > sdf(b.x, b.y, b.z);
    };

    // Enqueue initial candidates
    std::priority_queue<Vector3UZ, std::vector<Vector3UZ>, decltype(compare)>
        trial(compare);
    ForEachIndex(markers.Size(), [&](size_t i, size_t j, size_t k) {
        if (markers(i, j, k) == KNOWN)
        {
            return;
        }

        if (i > 0 && markers(i - 1, j, k) == KNOWN)
        {
            trial.push(Vector3UZ{ i, j, k });
            markers(i, j, k) = TRIAL;
            return;
        }

        if (i + 1 < size.x && markers(i + 1, j, k) == KNOWN)
        {
            trial.push(Vector3UZ{ i, j, k });
            markers(i, j, k) = TRIAL;
            return;
        }

        if (j > 0 && markers(i, j - 1, k) == KNOWN)
        {
            trial.push(Vector3UZ{ i, j, k });
            markers(i, j, k) = TRIAL;
            return;
        }

        if (j + 1 < size.y && markers(i, j + 1, k) == KNOWN)
        {
            trial.push(Vector3UZ{ i, j, k });
            markers(i, j, k) = TRIAL;
            return;
        }

        if (k > 0 && markers(i, j, k - 1) == KNOWN)
        {
            trial.push(Vector3UZ{ i, j, k });
            markers(i, j, k) = TRIAL;
            return;
        }

        if (k + 1 < size.z && markers(i, j, k + 1) == KNOWN)
        {
            trial.push(Vector3UZ{ i, j, k });
            markers(i, j, k) = TRIAL;
            return;
        }
    });

    // Propagate
    while (!trial.empty())
    {
        const Vector3UZ idx = trial.top();
        trial.pop();

        const size_t i = idx.x;
        const size_t j = idx.y;
        const size_t k = idx.z;

        if (sdf(i, j, k) > maxDistance)
        {
            break;
        }

        Vector3D grad = Gradient3(sdf, gridSpacing, i, j, k).Normalized();

        double sum = 0.0;
        double count = 0.0;

        if (i > 0)
        {
            if (markers(i - 1, j, k) == KNOWN)
            {
                double weight = std::max(grad.x, 0.0) * invGridSpacing.x;

                // If gradient is zero, then just assign 1 to weight
                if (weight < std::numeric_limits<double>::epsilon())
                {
                    weight = 1.0;
                }

                sum += weight * output(i - 1, j, k);
                count += weight;
            }
            else if (markers(i - 1, j, k) == UNKNOWN)
            {
                markers(i - 1, j, k) = TRIAL;
                trial.push(Vector3UZ{ i - 1, j, k });
            }
        }

        if (i + 1 < size.x)
        {
            if (markers(i + 1, j, k) == KNOWN)
            {
                double weight = -std::min(grad.x, 0.0) * invGridSpacing.x;

                // If gradient is zero, then just assign 1 to weight
                if (weight < std::numeric_limits<double>::epsilon())
                {
                    weight = 1.0;
                }

                sum += weight * output(i + 1, j, k);
                count += weight;
            }
            else if (markers(i + 1, j, k) == UNKNOWN)
            {
                markers(i + 1, j, k) = TRIAL;
                trial.push(Vector3UZ{ i + 1, j, k });
            }
        }

        if (j > 0)
        {
            if (markers(i, j - 1, k) == KNOWN)
            {
                double weight = std::max(grad.y, 0.0) * invGridSpacing.y;

                // If gradient is zero, then just assign 1 to weight
                if (weight < std::numeric_limits<double>::epsilon())
                {
                    weight = 1.0;
                }

                sum += weight * output(i, j - 1, k);
                count += weight;
            }
            else if (markers(i, j - 1, k) == UNKNOWN)
            {
                markers(i, j - 1, k) = TRIAL;
                trial.push(Vector3UZ{ i, j - 1, k });
            }
        }

        if (j + 1 < size.y)
        {
            if (markers(i, j + 1, k) == KNOWN)
            {
                double weight = -std::min(grad.y, 0.0) * invGridSpacing.y;

                // If gradient is zero, then just assign 1 to weight
                if (weight < std::numeric_limits<double>::epsilon())
                {
                    weight = 1.0;
                }

                sum += weight * output(i, j + 1, k);
                count += weight;
            }
            else if (markers(i, j + 1, k) == UNKNOWN)
            {
                markers(i, j + 1, k) = TRIAL;
                trial.push(Vector3UZ{ i, j + 1, k });
            }
        }

        if (k > 0)
        {
            if (markers(i, j, k - 1) == KNOWN)
            {
                double weight = std::max(grad.z, 0.0) * invGridSpacing.z;

                // If gradient is zero, then just assign 1 to weight
                if (weight < std::numeric_limits<double>::epsilon())
                {
                    weight = 1.0;
                }

                sum += weight * output(i, j, k - 1);
                count += weight;
            }
            else if (markers(i, j, k - 1) == UNKNOWN)
            {
                markers(i, j, k - 1) = TRIAL;
                trial.push(Vector3UZ{ i, j, k - 1 });
            }
        }

        if (k + 1 < size.z)
        {
            if (markers(i, j, k + 1) == KNOWN)
            {
                double weight = -std::min(grad.z, 0.0) * invGridSpacing.z;

                // If gradient is zero, then just assign 1 to weight
                if (weight < std::numeric_limits<double>::epsilon())
                {
                    weight = 1.0;
                }

                sum += weight * output(i, j, k + 1);
                count += weight;
            }
            else if (markers(i, j, k + 1) == UNKNOWN)
            {
                markers(i, j, k + 1) = TRIAL;
                trial.push(Vector3UZ{ i, j, k + 1 });
            }
        }

        assert(count > 0.0);

        output(i, j, k) = sum / count;
        markers(i, j, k) = KNOWN;
    }
}
}  // namespace CubbyFlow