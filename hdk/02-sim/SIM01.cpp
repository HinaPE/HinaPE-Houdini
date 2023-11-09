#include "SIM01.h"

#include <UT/UT_DSOVersion.h>
#include <PRM/PRM_Include.h>
#include <SIM/SIM_PRMShared.h>
#include <SIM/SIM_DopDescription.h>

SIM01::SIM01(const SIM_DataFactory *factory) : SIM_Data(factory), SIM_OptionsUser(this) {}
SIM01::~SIM01() = default;

void
initializeSIM(void *)
{
	IMPLEMENT_DATAFACTORY(SIM01);
}

auto SIM01::getSIM01Description() -> const SIM_DopDescription *
{
	static PRM_Name theTest("test", "Test");

	static PRM_Template theTemplates[] = {
			PRM_Template(PRM_FLT_J,	1, &theTest, PRMoneDefaults),
			PRM_Template()
	};

	static SIM_DopDescription theDopDescription(true,
												"hina_sim01",	// operator name
												"SIM 01",	// English name
												"SIM01",		// default data name
												classname(),
												theTemplates);

	return &theDopDescription;
}