#ifndef INC_03_SOLVER_SOLVER01_H
#define INC_03_SOLVER_SOLVER01_H

#include <SIM/SIM_Solver.h>
#include <SIM/SIM_DataUtils.h>
#include <SIM/SIM_OptionsUser.h>

class Solver01 : public SIM_Solver, public SIM_OptionsUser
{
public:
	GETSET_DATA_FUNCS_F("test", Test);

protected:
	explicit Solver01(const SIM_DataFactory *factory);
	~Solver01() override;
	auto solveObjectsSubclass(SIM_Engine &engine, SIM_ObjectArray &objects, SIM_ObjectArray &newobjects, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep) -> SIM_Result override;

private:
	static auto getSolver01Description() -> const SIM_DopDescription *;

DECLARE_STANDARD_GETCASTTOTYPE();
DECLARE_DATAFACTORY(Solver01, SIM_Solver, "Solver 01", getSolver01Description());
};

#endif //INC_03_SOLVER_SOLVER01_H
