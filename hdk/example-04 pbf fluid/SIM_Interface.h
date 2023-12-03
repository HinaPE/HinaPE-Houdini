#ifndef SIM_INTERFACE_H
#define SIM_INTERFACE_H

#include <SIM/SIM_Engine.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_Time.h>

#include <SIM/SIM_SingleSolver.h>
#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_DopDescription.h>

#include <SIM/SIM_Utils.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_PositionSimple.h>

#include <PRM/PRM_Name.h>
#include <PRM/PRM_Template.h>
#include <PRM/PRM_Shared.h>
#include <PRM/PRM_Default.h>

#include <vector>
#include <map>
#include <string>
#include <memory>
#include <iostream>

struct IParam
{
	std::map<std::string, fpreal> RealParams;
	std::map<std::string, UT_Vector3D> Vector3Params;
};

struct ISolver
{
	virtual void InitGeometry(SIM_Geometry *) = 0;
	virtual void SolveGeometry(SIM_Geometry *, const SIM_Time &) = 0;

	virtual void InitPosition(SIM_Position *) = 0;
	virtual void SolvePosition(SIM_Position *, const SIM_Time &) = 0;

	virtual ~ISolver() = default;

	// Implement these below in the cpp file of new solver
	static void ResetSolver();

	static std::unique_ptr<ISolver> Solver;
	static IParam Param;
};

template<typename Type>
Type *GetFromSubData(SIM_Object &object, const char *SubDataName)
{
	Type *value = SIM_DATA_GET(object, SubDataName, Type);
	if (!value)
		std::cout << "NULLPTR::" << SubDataName << "\n";
	return value;
}

#define ImplementNewSolverHeader(MySIMSolverName) \
class MySIMSolverName : public SIM_SingleSolver, public SIM_OptionsUser \
{ \
public: \
    NewSolverParameterGetSet \
    IParam params; \
private: \
    SIM_PBFSolver(const SIM_DataFactory *factory); \
    ~SIM_PBFSolver() override; \
    SIM_Result solveSingleObjectSubclass(SIM_Engine &engine, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep, bool newobject) override; \
    static const SIM_DopDescription *GetDescription(); \
DECLARE_STANDARD_GETCASTTOTYPE(); \
DECLARE_DATAFACTORY(MySIMSolverName, SIM_SingleSolver, SolverDescription, GetDescription()); \
};

#define ImplementNewSolverImpl(MySIMSolverName) \
MySIMSolverName::MySIMSolverName(const SIM_DataFactory *factory) : SIM_SingleSolver(factory), SIM_OptionsUser(this) { ISolver::ResetSolver(); } \
MySIMSolverName::~MySIMSolverName() { ISolver::Solver.reset(); }

#define ImplementInnerSolverRequired(MyInnerSolverName) \
std::unique_ptr<ISolver> ISolver::Solver = nullptr; \
IParam ISolver::Param; \
void ISolver::ResetSolver() { ISolver::Solver = std::make_unique<MyInnerSolverName>(); }

#endif //SIM_INTERFACE_H
