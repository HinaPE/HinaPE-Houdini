#include "solver.h"
#include "attribute_helper.h"

#include <GU/GU_PrimPoly.h>
#include <PRM/PRM_Include.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_DataFilter.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_Engine.h>
#include <SIM/SIM_Force.h>

#include <iostream>

class MySolver
{
public:
	MySolver() = default;
	~MySolver() = default;
};

Solver::Solver(const SIM_DataFactory *factory) : BaseClass(factory), SIM_OptionsUser(this) {}

auto Solver::solveSingleObjectSubclass(SIM_Engine &engine, SIM_Object &object, SIM_ObjectArray &object_array, const SIM_Time &timestep, bool newobject) -> SIM_Solver::SIM_Result
{
	bool build = false;

	SolverArgs args(*this, engine, object, object_array, timestep);

	return SIM_SOLVER_SUCCESS;
}

void Solver::makeEqualSubclass(const SIM_Data *source)
{
	BaseClass::makeEqualSubclass(source);
	const auto* s = SIM_DATA_CASTCONST(source, Solver);
	// do not share if the solver is being copied from another dopnet
	if (s && getOwnerNetwork() == s->getOwnerNetwork())
	{
		if (!s->wrapper)
			s->wrapper = std::make_shared<MySolverWrapper>();
		this->wrapper = s->wrapper;
	}
	else
		wrapper.reset();
}

SIM_Solver::SIM_Result MySolverWrapper::Init(SolverArgs &args)
{
	std::cout << "Init" << std::endl;
	SIM_GeometryCopy *geo = SIM_DATA_CREATE(args.object, "Geometry", SIM_GeometryCopy, SIM_DATA_RETURN_EXISTING | SIM_DATA_ADOPT_EXISTING_ON_DELETE);

	// Read Geometry
	{
		SIM_GeometryAutoWriteLock lock(geo);
		GU_Detail& gdp = lock.getGdp();

		auto prims = gdp.primitives();
		auto prim_count = prims.entries();

		GA_ROHandleF density = PointAttribute<GA_ROHandleF>(gdp, "density");
		if (density.isValid())
		{
			for (size_t i = 0; i < gdp.getNumPoints(); ++i)
			{
				auto offset = gdp.pointOffset(i);
				auto pos = gdp.getPos3(offset);

				// set to your own solver
			}
		}
	}

	// External Collision may come from affectors
	SIM_ObjectArray affectors;
	args.object.getAffectors(affectors, "SIM_RelationshipSource");
	auto n = affectors.entries();

	return SIM_Solver::SIM_SOLVER_SUCCESS;
}

SIM_Solver::SIM_Result MySolverWrapper::Solve(SolverArgs &args)
{
	return SIM_Solver::SIM_SOLVER_SUCCESS;
}
