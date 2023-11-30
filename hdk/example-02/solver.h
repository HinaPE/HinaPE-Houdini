#ifndef EXAMPLE_01_SOLVER_H
#define EXAMPLE_01_SOLVER_H

#include <SIM/SIM_SingleSolver.h>
#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Utils.h>

/// Very Simple Gravity and Collision With Plane Example
class GravityCollisionSolver : public SIM_SingleSolver, public SIM_OptionsUser
{
public:
	GETSET_DATA_FUNCS_V3("my_gravity", MyGravity);
	GETSET_DATA_FUNCS_F("my_plane_level", MyPlaneLevel);

private:
	GravityCollisionSolver(const SIM_DataFactory *factory) : SIM_SingleSolver(factory), SIM_OptionsUser(this) {}
	~GravityCollisionSolver() override = default;
	SIM_Result solveSingleObjectSubclass(SIM_Engine &engine, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep, bool newobject) override;
	static const SIM_DopDescription* GetDescription();

DECLARE_STANDARD_GETCASTTOTYPE();
DECLARE_DATAFACTORY(GravityCollisionSolver, SIM_SingleSolver, "Gravity Collision Solver Description", GetDescription());


// ==================== Custom Field ====================
protected:
	void init_gravity_effect(SIM_Object &obj);
	void solve_gravity_effect(SIM_Object &obj, const SIM_Time &dt) const;

	static bool NeedReBuild;
};

/// A simple Velocity SubData example
class VelocityData : public SIM_Data, public SIM_OptionsUser
{
public:
	GETSET_DATA_FUNCS_V3("my_velocity", MyVelocity)

private:
	VelocityData(const SIM_DataFactory *factory) : SIM_Data(factory), SIM_OptionsUser(this) {}
	~VelocityData() override = default;
	static const SIM_DopDescription* GetDescription();

DECLARE_STANDARD_GETCASTTOTYPE();
DECLARE_DATAFACTORY(VelocityData, SIM_Data, "My Velocity Data", GetDescription());
};


/// A simple Logger
#include <vector>
#include <string>
static struct Logger
{
	SIM_Solver::SIM_Result report() const { return log.empty() ? SIM_Solver::SIM_SOLVER_SUCCESS : SIM_Solver::SIM_SOLVER_FAIL; }
	void reset() { log.clear(); }
	void error_nullptr(const std::string& type) { log.emplace_back("NullPointerError::" + type); }

	std::vector<std::string> log;
} Log;

#endif //EXAMPLE_01_SOLVER_H
