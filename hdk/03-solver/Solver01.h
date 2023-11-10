#ifndef INC_03_SOLVER_SOLVER01_H
#define INC_03_SOLVER_SOLVER01_H

#include <SIM/SIM_Solver.h>
#include <SIM/SIM_SingleSolver.h>
#include <SIM/SIM_DataUtils.h>
#include <SIM/SIM_OptionsUser.h>

class Solver01 : public SIM_SingleSolver, public SIM_OptionsUser
{
protected:
	explicit Solver01(const SIM_DataFactory *factory);
	~Solver01() override;
	auto solveSingleObjectSubclass(SIM_Engine &engine, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep, bool newobject) -> SIM_Result override;

private:
	static auto getSolver01Description() -> const SIM_DopDescription *;

public:
DECLARE_STANDARD_GETCASTTOTYPE();
DECLARE_DATAFACTORY(Solver01, SIM_SingleSolver, "Solver 01", getSolver01Description());
	GETSET_DATA_FUNCS_F("test", Test);
};

#include <SIM/SIM_Geometry.h>

class MyGeometry : public SIM_Geometry
{
protected:
	void initializeSubclass() override;
	auto getGeometrySubclass() const -> GU_ConstDetailHandle override;
};

#endif //INC_03_SOLVER_SOLVER01_H
