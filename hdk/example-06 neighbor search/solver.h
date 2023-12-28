#ifndef EXAMPLE_01_SOLVER_H
#define EXAMPLE_01_SOLVER_H

#include <SIM/SIM_SingleSolver.h>
#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Utils.h>

#include <UT/UT_ParallelUtil.h>

/// Very Simple Gravity and Collision With Plane Example
class NeighborSearchSolver : public SIM_SingleSolver, public SIM_OptionsUser
{
public:
	GETSET_DATA_FUNCS_F("neighbor_radius", NeighborRadius);

private:
	NeighborSearchSolver(const SIM_DataFactory *factory) : SIM_SingleSolver(factory), SIM_OptionsUser(this) {}
	~NeighborSearchSolver() override = default;
	SIM_Result solveSingleObjectSubclass(SIM_Engine &engine, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep, bool newobject) override;
	static const SIM_DopDescription* GetDescription();

DECLARE_STANDARD_GETCASTTOTYPE();
DECLARE_DATAFACTORY(NeighborSearchSolver, SIM_SingleSolver, "Neighbor Search Solver Description", GetDescription());
};

#endif //EXAMPLE_01_SOLVER_H
