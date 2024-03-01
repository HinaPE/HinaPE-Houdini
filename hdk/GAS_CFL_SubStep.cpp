#include "GAS_CFL_SubStep.h"

#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_DopDescription.h>
#include <PRM/PRM_Template.h>
#include <PRM/PRM_Default.h>
#include <Base/SIM_Hina_Particles.h>
#include <Boundary/SIM_Hina_Particles_Akinci.h>
#include <Boundary/SIM_Hina_Particles_Bender.h>

const SIM_DopDescription *GAS_CFL_SubStep::getDopDescription()
{
	static PRM_Name MAX_SUBSTEP("MAX_SUBSTEP", "MAX_SUBSTEP");
	static PRM_Default DefaultMAX_SUBSTEP{2};

	static std::array<PRM_Template, 2> PRMS{
			PRM_Template(PRM_INT, 1, &MAX_SUBSTEP, &DefaultMAX_SUBSTEP),
			PRM_Template()
	};

	static SIM_DopDescription DESC(true,
								   "CFL_SubStep",
								   "CFL_SubStep",
								   "CFL_SubStep",
								   classname(),
								   PRMS.data());
	DESC.setDefaultUniqueDataName(true);
	return &DESC;
}

SIM_Solver::SIM_Result GAS_CFL_SubStep::solveObjectsSubclass(SIM_Engine &engine, SIM_ObjectArray &objects, SIM_ObjectArray &newobjects, SIM_ObjectArray &feedbacktoobjects, const SIM_Time &timestep)
{
	// TODO: Implement CFL substep
	float mas_step = getMAX_SUBSTEP();
	fpreal sub_time = timestep / mas_step;
	for (int i = 0; i < mas_step; ++i)
	{
		SIM_Solver::SIM_Result res = GAS_SubStep::solveObjectsSubclass(engine, objects, newobjects, feedbacktoobjects, sub_time);
		if (res != SIM_Solver::SIM_SOLVER_SUCCESS)
			return res;
	}
	for (int j = 0; j < objects.size(); ++j)
	{
		SIM_Object *obj = objects(j);
		SIM_Hina_Particles *particles = SIM_DATA_GET(*obj, SIM_Hina_Particles::DATANAME, SIM_Hina_Particles);
		if (particles && particles->x != nullptr)
			particles->commit();

		std::vector<SIM_Hina_Particles_Akinci *> akinci_boundaries = FetchAllAkinciBoundaries(obj);
		for (auto &akinci_boundary: akinci_boundaries)
			if (akinci_boundary && akinci_boundary->x != nullptr)
				akinci_boundary->commit();
	}
	return SIM_Solver::SIM_SOLVER_SUCCESS;
}
