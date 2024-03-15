#include "GAS_Hina_Test.h"

#include <Base/utils.h>

//#include "DataTypes.h"
//#include "Field.h"
//#include "Collision/CollisionDetectionBroadPhase.h"

GAS_HINA_SUBSOLVER_IMPLEMENT(
		Test,
		true,
		false,
)
void GAS_Hina_Test::_init() {}
void GAS_Hina_Test::_makeEqual(const GAS_Hina_Test *src) {}
bool GAS_Hina_Test::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	static bool init = false;
	if (!init)
	{
//		using namespace dyno;
//		CollisionDetectionBroadPhase<DataType3f> collision;
	}
	return true;
}
