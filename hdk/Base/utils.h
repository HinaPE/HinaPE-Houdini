#ifndef HINAPE_UTILS_H
#define HINAPE_UTILS_H

/**
 * Utils for Houdini HDK Dev
 */

#include <SIM/SIM_Object.h>
#include <SIM/SIM_Geometry.h>

template <typename real, typename Vector>
std::pair<std::vector<Vector>, std::vector<size_t>> ReadTriangleMeshFromGeometry(SIM_Object *obj, const char *DATANAME, Vector &OUT_pos)
{
	std::pair<std::vector<Vector>, std::vector<size_t>> res;

	SIM_Geometry *SOPGeometry = SIM_DATA_GET(*obj, DATANAME, SIM_Geometry);
	if (!SOPGeometry)
		throw std::runtime_error("No geometry found for " + std::string(DATANAME));

	SIM_GeometryAutoReadLock lock(SOPGeometry);
	const GU_Detail *gdp = lock.getGdp();

	Vector center_of_mass{0, 0, 0};
	{
		GA_Offset pt_off;
		GA_FOR_ALL_PTOFF(gdp, pt_off)
			{
				UT_Vector3 _pos = gdp->getPos3(pt_off);
				Vector pos = {_pos.x(), _pos.y(), _pos.z()};
				center_of_mass += pos;
			}
		center_of_mass /= (real)gdp->getNumPoints();
	}
	OUT_pos = center_of_mass;

	std::vector<Vector> &vertices = res.first;
	vertices.resize(gdp->getNumPoints());
	{
		GA_Offset pt_off;
		GA_FOR_ALL_PTOFF(gdp, pt_off)
			{
				GA_Size pt_idx = gdp->pointIndex(pt_off);
				UT_Vector3 _pos = gdp->getPos3(pt_off);
				Vector pos = {_pos.x(), _pos.y(), _pos.z()};
				vertices[pt_idx] = pos - center_of_mass;
			}
	}

	std::vector<size_t> &indices = res.second;
	{
		const GEO_Primitive *prim;
		GA_FOR_ALL_PRIMITIVES(gdp, prim)
		{
			GA_Size prim_vertex_count = prim->getVertexCount();
			if (prim_vertex_count == 3)
			{
				for (GA_Size i = 0; i < prim_vertex_count; i += 3)
				{
					GA_Offset pt_off = prim->getPointOffset(i);
					GA_Index pt_idx = gdp->pointIndex(pt_off);
					indices.emplace_back(pt_idx);

					pt_off = prim->getPointOffset(i + 2); // Important: invert the indices
					pt_idx = gdp->pointIndex(pt_off);
					indices.emplace_back(pt_idx);

					pt_off = prim->getPointOffset(i + 1);
					pt_idx = gdp->pointIndex(pt_off);
					indices.emplace_back(pt_idx);
				}
			} else if (prim_vertex_count == 4)
			{
				GA_Offset pt_off = prim->getPointOffset(0);
				GA_Index pt_idx = gdp->pointIndex(pt_off);
				indices.emplace_back(pt_idx);

				pt_off = prim->getPointOffset(2); // Important: invert the indices
				pt_idx = gdp->pointIndex(pt_off);
				indices.emplace_back(pt_idx);

				pt_off = prim->getPointOffset(1);
				pt_idx = gdp->pointIndex(pt_off);
				indices.emplace_back(pt_idx);

				pt_off = prim->getPointOffset(0);
				pt_idx = gdp->pointIndex(pt_off);
				indices.emplace_back(pt_idx);

				pt_off = prim->getPointOffset(3); // Important: invert the indices
				pt_idx = gdp->pointIndex(pt_off);
				indices.emplace_back(pt_idx);

				pt_off = prim->getPointOffset(2);
				pt_idx = gdp->pointIndex(pt_off);
				indices.emplace_back(pt_idx);
			} else
				throw std::runtime_error("Only triangle and quad are supported");
		}
	}

	return res;
}

#endif //HINAPE_UTILS_H
