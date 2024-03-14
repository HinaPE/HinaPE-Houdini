#ifndef HINAPE_GEOMETRY_H
#define HINAPE_GEOMETRY_H

#include <CUDA_CubbyFlow/Core/Geometry/SurfaceSet.hpp>
#include <CUDA_CubbyFlow/Core/Geometry/ImplicitSurfaceSet.hpp>
#include <CUDA_CubbyFlow/Core/Geometry/SurfaceToImplicit.hpp>
#include <CUDA_CubbyFlow/Core/Geometry/TriangleMesh3.hpp>
#include <CUDA_CubbyFlow/Core/Geometry/RigidBodyCollider.hpp>

namespace HinaPE
{
template<typename real, typename Vector3, typename Quaternion>
struct ISurface
{
	/**
	 * make sure the vertices and indices are valid [Triangle Surface]
	 */
	ISurface(const std::vector<Vector3> &vertices, const std::vector<size_t> &indices)
	{
		CubbyFlow::TriangleMesh3::PointArray points;
		CubbyFlow::TriangleMesh3::IndexArray point_indices;
		for (size_t i = 0; i < vertices.size(); i++)
			points.Append(CubbyFlow::Vector3D{vertices[i].x(), vertices[i].y(), vertices[i].z()});
		for (size_t i = 0; i < indices.size(); i += 3)
			point_indices.Append(CubbyFlow::Vector3UZ{indices[i], indices[i + 1], indices[i + 2]});
		surface = CubbyFlow::TriangleMesh3::GetBuilder().WithPoints(points).WithPointIndices(point_indices).MakeShared();
		implicit_surface = CubbyFlow::SurfaceToImplicit3::GetBuilder().WithSurface(surface).MakeShared();
		rigid_body_collider = CubbyFlow::RigidBodyCollider3::GetBuilder().WithSurface(surface).MakeShared();
	}

	void update_transform(const Vector3 &translation, const Quaternion &rotation)
	{
		rigid_body_collider->GetSurface()->transform
				= CubbyFlow::Transform3{
				CubbyFlow::Vector3D{translation.x(), translation.y(), translation.z()},
				CubbyFlow::QuaternionD{rotation.x(), rotation.y(), rotation.z(), rotation.w()}
		};
	}

	real signed_distance(const Vector3 &point) const
	{
		CubbyFlow::Vector3D point_cf{point.x(), point.y(), point.z()};

		double _distance = surface->ClosestDistance(point_cf);
		CubbyFlow::Vector3D _point = surface->ClosestPoint(point_cf);
		CubbyFlow::Vector3D _normal = surface->ClosestNormal(point_cf);

		(point_cf - _point).Dot(_normal) < 0.0 ? _distance = -_distance : _distance = _distance;

		return _distance;
	}

private:
	CubbyFlow::Surface3Ptr surface;
	CubbyFlow::ImplicitSurface3Ptr implicit_surface;
	CubbyFlow::RigidBodyCollider3Ptr rigid_body_collider;
};
}

#endif //HINAPE_GEOMETRY_H
