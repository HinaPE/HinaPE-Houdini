#ifndef HINAPE_HOUDINI_SOLVER_H
#define HINAPE_HOUDINI_SOLVER_H

#include <SIM/SIM_SingleSolver.h>
#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_Engine.h>
#include <GU/GU_PrimPoly.h>
#include <PRM/PRM_Include.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_DataFilter.h>

class Solver : public SIM_SingleSolver, public SIM_OptionsUser
{
protected:
	explicit Solver(const SIM_DataFactory* factory)
			: BaseClass(factory)
			, SIM_OptionsUser(this)
	{
	};

	~Solver() override {};

	SIM_Result solveSingleObjectSubclass(SIM_Engine &engine, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep, bool newobject) override;

private:
	static const SIM_DopDescription* getDopDescription()
	{
		static PRM_Template theTemplates[] = { PRM_Template() };

		static SIM_DopDescription theDopDescription(
				true, "_EXAMPLE_SOLVER_", "_EXAMPLE_SOLVER_", SIM_SOLVER_DATANAME, classname(), theTemplates);

		return &theDopDescription;
	}

	DECLARE_STANDARD_GETCASTTOTYPE();
	DECLARE_DATAFACTORY(Solver, SIM_SingleSolver, "_EXAMPLE_SOLVER_", getDopDescription());
};

#endif //HINAPE_HOUDINI_SOLVER_H
