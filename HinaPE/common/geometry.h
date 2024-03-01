#ifndef HINAPE_GEOMETRY_H
#define HINAPE_GEOMETRY_H

#include <CUDA_CubbyFlow/Core/Geometry/Surface.hpp>
#include <CUDA_CubbyFlow/Core/Geometry/ImplicitSurface.hpp>

namespace HinaPE
{
template<typename real, typename Vector3>
struct ITriangleMesh
{
	std::vector<Vector3> vertices;
	std::vector<size_t> indices;
};

template<typename real, typename Vector3, typename ScalarArray, typename Vector3Array>
struct IImplicitSDF
{

};
}

#endif //HINAPE_GEOMETRY_H
