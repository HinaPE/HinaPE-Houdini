#ifndef HINAPE_UTIL_FCL_H
#define HINAPE_UTIL_FCL_H

#include "fcl/fcl.h"
#include <GU/GU_Detail.h>
#include <GU/GU_PrimPoly.h>
#include <GEO/GEO_Macros.h>
#include <SIM/SIM_Position.h>

namespace HinaPE
{

// Note: Make sure Primitive is Triangle
std::shared_ptr<fcl::CollisionObjectf> AsFCLCollider(const GU_Detail &gdp, const SIM_Position &position)
{
	std::vector<fcl::Vector3f> vertices;
	std::vector<fcl::Triangle> triangles;

	// code to set the vertices and triangles
	GA_Size point_size = gdp.getNumPoints();
	for (int i = 0; i < point_size; ++i)
	{
		GA_Offset offset = gdp.pointOffset(i);
		const UT_Vector3 &pos = gdp.getPos3(offset);
		vertices.emplace_back(pos.x(), pos.y(), pos.z());
	}

	GA_Size prim_size = gdp.getNumPrimitives();
	for (int i = 0; i < prim_size; ++i)
	{
		GA_Offset offset = gdp.primitiveOffset(i);
		const GA_Primitive *prim = gdp.getGEOPrimitive(offset);
		if (prim->getTypeId() == GA_PRIMPOLY)
		{
			const GA_Primitive *poly = prim;
			int vertex_count = poly->getVertexCount();
			for (int j = 0; j < vertex_count; ++j)
			{
				int v0 = poly->getVertexOffset(j);
				int v1 = poly->getVertexOffset((j + 1) % vertex_count);
				int v2 = poly->getVertexOffset((j + 2) % vertex_count);
				GA_Offset p0 = gdp.vertexPoint(v0);
				GA_Offset p1 = gdp.vertexPoint(v1);
				GA_Offset p2 = gdp.vertexPoint(v2);
				triangles.emplace_back(p0, p1, p2);
			}
		}
	}

	// BVHModel is a template class for mesh geometry, for default OBBRSS template is used
	typedef fcl::BVHModel<fcl::OBBRSSf> Model;
	std::shared_ptr<Model> geom = std::make_shared<Model>();

	// add the mesh data into the BVHModel structure
	geom->beginModel();
	geom->addSubModel(vertices, triangles);
	geom->endModel();

	fcl::Transform3f tf = fcl::Transform3f::Identity();
	{
		UT_Matrix4D mat;
		position.getTransform(mat);
		fcl::Matrix3f linear;
		linear << mat[0][0], mat[0][1], mat[0][2],
				mat[1][0], mat[1][1], mat[1][2],
				mat[2][0], mat[2][1], mat[2][2];
		linear.transpose();
		tf.linear() = linear;

		UT_Vector3 pos;
		position.getPosition(pos);
		tf.translation() = fcl::Vector3f(pos.x(), pos.y(), pos.z());
	}

	return std::make_shared<fcl::CollisionObjectf>(geom, tf);
}
std::shared_ptr<fcl::CollisionObjectf> AsFCLCollider(const GU_Detail *gdp, const SIM_Position *position) { return AsFCLCollider(*gdp, *position); }
std::shared_ptr<fcl::CollisionObjectf> AsFCLCollider(const GU_ConstDetailHandle gdh, const SIM_Position *position) { return AsFCLCollider(*gdh.gdp(), *position); }
}

#endif //HINAPE_UTIL_FCL_H
