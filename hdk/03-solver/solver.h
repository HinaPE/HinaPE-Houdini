#ifndef INC_03_SOLVER_SOLVER01_H
#define INC_03_SOLVER_SOLVER01_H

#include <SIM/SIM_SingleSolver.h>
#include <SIM/SIM_DataUtils.h>
#include <SIM/SIM_OptionsUser.h>

class HinaClothSolver : public SIM_SingleSolver, public SIM_OptionsUser
{
protected:
	explicit HinaClothSolver(const SIM_DataFactory *factory);
	~HinaClothSolver() override;
	auto solveSingleObjectSubclass(SIM_Engine &engine, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep, bool newobject) -> SIM_Result override;

private:
	static auto getSolver01Description() -> const SIM_DopDescription *;


public:
DECLARE_STANDARD_GETCASTTOTYPE();
DECLARE_DATAFACTORY(HinaClothSolver, SIM_SingleSolver, "Hina Cloth Solver", getSolver01Description());
	GETSET_DATA_FUNCS_F("test", Test);
};

#endif //INC_03_SOLVER_SOLVER01_H
