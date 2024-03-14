#include "GAS_Hina_Test.h"

#include <Base/utils.h>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		Test,
		true,
		false,
)
void GAS_Hina_Test::_init()
{
	this->InnerPtr = nullptr;
}
void GAS_Hina_Test::_makeEqual(const GAS_Hina_Test *src)
{
	this->InnerPtr = src->InnerPtr;
}
bool GAS_Hina_Test::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	if (!InnerPtr)
	{
		UT_Vector3 pos;
		std::pair<std::vector<Vector>, std::vector<size_t>> triangle_mesh_info = ReadTriangleMeshFromGeometry<double, Vector>(obj, SIM_GEOMETRY_DATANAME, pos);
		const std::vector<Vector> &V = triangle_mesh_info.first;
		const std::vector<size_t> &I = triangle_mesh_info.second;
		this->InnerPtr = std::make_shared<Surface>(V, I);
		InnerPtr->update_transform(pos, Quaternion{0, 0, 0 ,1});
	}
	return true;
}
