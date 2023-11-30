#include "solver.h"

#include <PRM/PRM_Name.h>
#include <PRM/PRM_Template.h>
#include <PRM/PRM_Shared.h>
#include <PRM/PRM_Default.h>

#include <SIM/SIM_Engine.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_GeometryCopy.h>

#include <GU/GU_Detail.h>
#include <GA/GA_Handle.h>

#include <array>
#include <iostream>


const SIM_DopDescription *PBDClothSolver::GetDescription()
{
	static PRM_Name gravity("gravity", "Gravity");
	static std::array<PRM_Default, 3> gravity_default = {0, -9.8, 0};

	static PRM_Name stiffness("stiffness", "Stiffness");
	static PRM_Default stiffness_default(-10);

	static std::array<PRM_Template, 3> PRMS{
			PRM_Template(PRM_FLT_J, 3, &gravity, gravity_default.data()),
			PRM_Template(PRM_FLT_J, 1, &stiffness, &stiffness_default),
			PRM_Template()
	};

	static SIM_DopDescription DESC(true,
								   "pbd_cloth_solver",
								   "PBD Cloth Solver",
								   "PBDClothSolver",
								   classname(),
								   PRMS.data());
	return &DESC;
}

SIM_Solver::SIM_Result PBDClothSolver::solveSingleObjectSubclass(SIM_Engine &, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep, bool newobject)
{
	Log.reset();

	static bool NeedReBuild = true;

	if (NeedReBuild || newobject)
	{
		init(object);
		NeedReBuild = false;
	} else
	{
		solve(object, timestep);
	}

	return Log.report();
}

void PBDClothSolver::init(SIM_Object &obj)
{
	SIM_GeometryCopy *geo;
	geo = SIM_DATA_CREATE(obj, "Geometry", SIM_GeometryCopy,
						  SIM_DATA_RETURN_EXISTING | SIM_DATA_ADOPT_EXISTING_ON_DELETE);
	if (!geo)
		Log.error_nullptr("INIT::SIM_GeometryCopy");

	std::cout << "Inited" << '\n';
}

void PBDClothSolver::solve(SIM_Object &obj, const SIM_Time &dt)
{
	SIM_GeometryCopy *geo;
	geo = SIM_DATA_GET(obj, "Geometry", SIM_GeometryCopy);
	if (!geo)
		Log.error_nullptr("SOLVE::SIM_GeometryCopy");

	{
		SIM_GeometryAutoWriteLock lock(geo);
		GU_Detail& gdp = lock.getGdp();

		GA_ROHandleF stiffness(gdp.findPointAttribute("stiffness"));
		if (stiffness.isValid())
		{

		}
	}

	std::cout << "Solved" << '\n';
}
