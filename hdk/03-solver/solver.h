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
	static auto getDescription() -> const SIM_DopDescription *;


public:
DECLARE_STANDARD_GETCASTTOTYPE();
DECLARE_DATAFACTORY(HinaClothSolver, SIM_SingleSolver, "Hina Cloth Solver", getDescription());
	GETSET_DATA_FUNCS_F("test", Test);
};

class HinaClothSolver2 : public SIM_Solver, public SIM_OptionsUser
{
protected:
	explicit HinaClothSolver2(const SIM_DataFactory *factory);
	~HinaClothSolver2() override;

	auto solveObjectsSubclass(SIM_Engine &engine, SIM_ObjectArray &objects, SIM_ObjectArray &newobjects, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep) -> SIM_Result override;

private:
	static auto getDescription() -> const SIM_DopDescription *;
DECLARE_STANDARD_GETCASTTOTYPE();
DECLARE_DATAFACTORY(HinaClothSolver2, SIM_Solver, "Hina Cloth Solver 2", getDescription());
	GETSET_DATA_FUNCS_F("test", Test);
};

#endif //INC_03_SOLVER_SOLVER01_H
