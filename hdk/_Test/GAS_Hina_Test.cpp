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
		std::pair<std::vector<UT_Vector3>, std::vector<size_t>> triangle_mesh_info = ReadTriangleMeshFromGeometry(obj, SIM_GEOMETRY_DATANAME);
		const std::vector<UT_Vector3> &V = triangle_mesh_info.first;
		const std::vector<size_t> &I = triangle_mesh_info.second;
		this->InnerPtr = std::make_shared<Surface>(V, I);
	}

	SIM_Position *pos = SIM_DATA_GET(*obj, SIM_POSITION_DATANAME, SIM_Position);
	if (pos)
	{
		UT_Vector3 p;
		pos->getPosition(p);
		InnerPtr->update_transform(p, UT_QuaternionD{0, 0, 0 ,1});
		std::cout << InnerPtr->SignedDistance(UT_Vector3D{0, 0, 0}) << std::endl;
	}
	return true;
}
