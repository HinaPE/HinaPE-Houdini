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

#include <memory>

class MySolver;
class Solver;

struct SolverArgs
{
	Solver &solverObject;
	SIM_Engine &engine;
	SIM_Object &object;
	SIM_ObjectArray &objectArray;
	const SIM_Time &timestep;

	SolverArgs(Solver &solverObject, SIM_Engine &engine, SIM_Object &object, SIM_ObjectArray &objectArray, const SIM_Time &timestep)
			: solverObject(solverObject), engine(engine), object(object), objectArray(objectArray), timestep(timestep)
	{}
};

struct MySolverWrapper
{
	MySolverWrapper() = default;
	std::unique_ptr<MySolver> my_solver;

	SIM_Solver::SIM_Result Init(SolverArgs &args);
	SIM_Solver::SIM_Result Solve(SolverArgs &args);
};

class Solver : public SIM_SingleSolver, public SIM_OptionsUser
{
protected:
	explicit Solver(const SIM_DataFactory *factory);
	~Solver() override = default;
	auto solveSingleObjectSubclass(SIM_Engine &engine, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep, bool newobject) -> SIM_Result override;
	void makeEqualSubclass(const SIM_Data *source) override;
	mutable std::shared_ptr<MySolverWrapper> wrapper = nullptr;

private:
	static auto getDopDescription() -> const SIM_DopDescription *
	{
		static PRM_Template theTemplates[] = {PRM_Template()};

		static SIM_DopDescription theDopDescription(
				true, "HinaClothSolver", "HinaClothSolver", SIM_SOLVER_DATANAME, classname(), theTemplates);

		return &theDopDescription;
	}

	DECLARE_STANDARD_GETCASTTOTYPE();
	DECLARE_DATAFACTORY(Solver, SIM_SingleSolver, "HinaClothSolver", getDopDescription());
};

#endif //HINAPE_HOUDINI_SOLVER_H
