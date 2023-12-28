#ifndef HINAPE_SIM_PBF_H
#define HINAPE_SIM_PBF_H

#include <SIM/SIM_SingleSolver.h>
#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Utils.h>

class SIM_PBFSolver : public SIM_SingleSolver, public SIM_OptionsUser
{
private:
	SIM_PBFSolver(const SIM_DataFactory *factory) : SIM_SingleSolver(factory), SIM_OptionsUser(this) {}
	~SIM_PBFSolver() override = default;
	SIM_Result solveSingleObjectSubclass(SIM_Engine &engine, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep, bool newobject) override;
	static const SIM_DopDescription* GetDescription();

	DECLARE_STANDARD_GETCASTTOTYPE();
	DECLARE_DATAFACTORY(SIM_PBFSolver, SIM_SingleSolver, "PBF Solver Description", GetDescription());
};

#endif //HINAPE_SIM_PBF_H
