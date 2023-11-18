#ifndef EXAMPLE_01_SOLVER_H
#define EXAMPLE_01_SOLVER_H

#include <SIM/SIM_Solver.h>
#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_DopDescription.h>

class MySolver : public SIM_Solver, public SIM_OptionsUser
{
protected:
	MySolver(const SIM_DataFactory *factory);
	~MySolver() override;

protected:
	SIM_Result solveObjectsSubclass(SIM_Engine &engine, SIM_ObjectArray &objects, SIM_ObjectArray &newobjects, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep) override;

private:
	static const SIM_DopDescription* GetDescription();

	DECLARE_STANDARD_GETCASTTOTYPE();
	DECLARE_DATAFACTORY(MySolver, SIM_Solver, "My Solver", GetDescription());
	GETSET_DATA_FUNCS_F("my_gravity", MyGravity);
};

#endif //EXAMPLE_01_SOLVER_H
