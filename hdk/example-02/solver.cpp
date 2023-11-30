#include "solver.h"

#include <PRM/PRM_Name.h>
#include <PRM/PRM_Template.h>
#include <PRM/PRM_Shared.h>
#include <PRM/PRM_Default.h>

#include <SIM/SIM_Engine.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_PositionSimple.h>
#include <SIM/SIM_GeometryCopy.h>

#include <GU/GU_Detail.h>
#include <GA/GA_Handle.h>

#include <array>
#include <iostream>

bool GravityCollisionSolver::NeedReBuild = true;

const SIM_DopDescription *GravityCollisionSolver::GetDescription()
{
	static PRM_Name my_gravity("my_gravity", "My Gravity");
	static std::array<PRM_Default, 3> my_gravity_default = {0, -9.8, 0};

	static PRM_Name my_plane_level("my_plane_level", "My Plane Level");
	static PRM_Default my_plane_level_default(-10);

	static std::array<PRM_Template, 3> PRMS{
			PRM_Template(PRM_XYZ_J, 3, &my_gravity, my_gravity_default.data()),
			PRM_Template(PRM_FLT_J, 1, &my_plane_level, &my_plane_level_default),
			PRM_Template()
	};

	static SIM_DopDescription DESC(true,
								   "gravity_collision_solver",
								   "Gravity Collision Solver",
								   "GravityCollisionSolver",
								   classname(),
								   PRMS.data());
	return &DESC;
}

const SIM_DopDescription *VelocityData::GetDescription()
{
	static PRM_Name my_velocity("my_velocity", "My Velocity");
	static std::array<PRM_Default, 3> my_velocity_default = {0, 0, 0};

	static std::array<PRM_Template, 2> PRMS{
			PRM_Template(PRM_XYZ, 3, &my_velocity, my_velocity_default.data()),
			PRM_Template()
	};

	static SIM_DopDescription DESC(true,
								   "my_velocity",
								   "My Velocity",
								   "MyVelocity",
								   classname(),
								   PRMS.data());
	return &DESC;
}

SIM_Solver::SIM_Result GravityCollisionSolver::solveSingleObjectSubclass(SIM_Engine &, SIM_Object &object, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep, bool newobject)
{
	Log.reset();

	if (NeedReBuild || newobject)
	{
		init_gravity_effect(object);
		NeedReBuild = false;
	} else
	{
		solve_gravity_effect(object, timestep);
	}

	return SIM_SOLVER_SUCCESS;
}

void GravityCollisionSolver::init_gravity_effect(SIM_Object &obj)
{
	SIM_PositionSimple *pos;
	pos = SIM_DATA_CREATE(obj, "Position", SIM_PositionSimple,
						  SIM_DATA_RETURN_EXISTING | SIM_DATA_ADOPT_EXISTING_ON_DELETE);
	if (!pos)
		Log.error_nullptr("INIT::SIM_PositionSimple");

	VelocityData *vel;
	vel = SIM_DATA_CREATE(obj, "VelocityData", VelocityData,
						  SIM_DATA_RETURN_EXISTING | SIM_DATA_ADOPT_EXISTING_ON_DELETE);
	if (!vel)
		Log.error_nullptr("INIT::VelocityData");
}

void GravityCollisionSolver::solve_gravity_effect(SIM_Object &obj, const SIM_Time &dt) const
{
	SIM_PositionSimple *pos;
	pos = SIM_DATA_GET(obj, "Position", SIM_PositionSimple);
	if (!pos)
		Log.error_nullptr("SOLVE::SIM_PositionSimple");

	VelocityData *vel;
	vel = SIM_DATA_GET(obj, "VelocityData", VelocityData);
	if (!vel)
		Log.error_nullptr("SOLVE::VelocityData");

	// Dead simple Semi-Euler
	UT_Vector3 p = pos->getPosition();
	UT_Vector3 v = vel->getMyVelocity();
	UT_Vector3 f = getMyGravity();

	v = v + dt * f; // assume mass = 1;
	p = p + dt * v;

	// Dead simple Floor Collision
	float floor_level = getMyPlaneLevel();
	if (p.y() < floor_level)
	{
		p.y() = floor_level;
		v.y() = -v.y();
	}

	vel->setMyVelocity(v);
	pos->setPosition(p);
}
