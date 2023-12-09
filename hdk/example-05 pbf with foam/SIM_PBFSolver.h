#ifndef SIM_PBFSolver_h
#define SIM_PBFSolver_h

#include <SIM/SIM_SingleSolver.h>
#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Utils.h>

/// PBF Example
class SIM_PBFSolver : public SIM_SingleSolver, public SIM_OptionsUser
{
public:
	GETSET_DATA_FUNCS_V3("gravity", Gravity);
	GETSET_DATA_FUNCS_I("dt", Dt);

	GET_GUIDE_FUNC_B(SIM_NAME_SHOWGUIDE, ShowGuide, true);
	GET_GUIDE_FUNC_V3(SIM_NAME_COLOR, Color, (1, 1, 1));
	GET_GUIDE_FUNC_V3("boundary", Boundary, (2, 2, 2));

private:
	SIM_PBFSolver(const SIM_DataFactory *factory) : SIM_SingleSolver(factory), SIM_OptionsUser(this) {}
	~SIM_PBFSolver() override = default;
	SIM_Result solveSingleObjectSubclass(SIM_Engine &engine, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep, bool newobject) override;
	static const SIM_DopDescription *GetDescription();

protected:
	SIM_Guide *createGuideObjectSubclass() const override;
	void buildGuideGeometrySubclass(const SIM_RootData &root, const SIM_Options &options, const GU_DetailHandle &gdh, UT_DMatrix4 *xform, const SIM_Time &t) const override;

DECLARE_STANDARD_GETCASTTOTYPE();
DECLARE_DATAFACTORY(SIM_PBFSolver, SIM_SingleSolver, "PBF Solver Description", GetDescription());
};

#endif //SIM_PBFSolver_h
