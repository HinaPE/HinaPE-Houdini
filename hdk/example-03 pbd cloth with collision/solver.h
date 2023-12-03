#ifndef EXAMPLE_01_SOLVER_H
#define EXAMPLE_01_SOLVER_H

#include <SIM/SIM_SingleSolver.h>
#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Utils.h>

/// PBD Cloth Example
class PBDClothSolver : public SIM_SingleSolver, public SIM_OptionsUser
{
public:
	GETSET_DATA_FUNCS_V3("gravity", Gravity);
	GETSET_DATA_FUNCS_F("stiffness", Stiffness);
	GETSET_DATA_FUNCS_I("constraint_iteration", ConstraintIteration);

private:
	PBDClothSolver(const SIM_DataFactory *factory) : SIM_SingleSolver(factory), SIM_OptionsUser(this) {}
	~PBDClothSolver() override = default;
	SIM_Result solveSingleObjectSubclass(SIM_Engine &engine, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep, bool newobject) override;
	static const SIM_DopDescription *GetDescription();

DECLARE_STANDARD_GETCASTTOTYPE();
DECLARE_DATAFACTORY(PBDClothSolver, SIM_SingleSolver, "PBD Cloth Solver Description", GetDescription());


// ==================== Custom Field ====================
protected:
	void init(SIM_Object &obj) const;
	void solve(SIM_Object &obj, const SIM_Time &dt) const;

public:
	struct DistanceConstraint
	{
		GA_Offset P1, P2;
		fpreal rest_length;
	};
	static std::vector<DistanceConstraint> DCs;
};


/// A simple Logger
#include <iostream>
#include <vector>
#include <string>

static struct Logger
{
	SIM_Solver::SIM_Result report()
	{
		for (const auto &l: log) std::cout << l << '\n';
		return log.empty() ? SIM_Solver::SIM_SOLVER_SUCCESS : SIM_Solver::SIM_SOLVER_FAIL;
	}
	void reset() { log.clear(); }
	void error_nullptr(const std::string &type) { log.emplace_back("NullPointerError::" + type); }

	std::vector<std::string> log;
} Log;

#endif //EXAMPLE_01_SOLVER_H
