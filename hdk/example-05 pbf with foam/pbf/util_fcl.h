#ifndef HINAPE_UTIL_FCL_H
#define HINAPE_UTIL_FCL_H

#include "fcl/fcl.h"
#include <GU/GU_Detail.h>
#include <GU/GU_PrimPoly.h>
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
				int v0 = poly->getVertexIndex(j);
				int v1 = poly->getVertexIndex((j + 1) % vertex_count);
				int v2 = poly->getVertexIndex((j + 2) % vertex_count);
				triangles.emplace_back(v0, v1, v2);
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

	fcl::Transform3f tf;
	tf.linear() = fcl::Matrix3f::Identity();
	{
		UT_Vector3 pos;
		position.getPosition(pos);
		tf.translation() = fcl::Vector3f(pos.x(), pos.y(), pos.z());

		UT_Matrix4D mat;
		position.getTransform(mat);
		fcl::Matrix3f linear;
		linear << mat[0][0], mat[0][1], mat[0][2],
				mat[1][0], mat[1][1], mat[1][2],
				mat[2][0], mat[2][1], mat[2][2];
		tf.linear() = linear;
	}

	return std::make_shared<fcl::CollisionObjectf>(geom, tf);
}
}

#endif //HINAPE_UTIL_FCL_H
