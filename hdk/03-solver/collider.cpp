#include "collider.h"

#include <PRM/PRM_Name.h>
#include <PRM/PRM_Template.h>
#include <PRM/PRM_Shared.h>

HinaCollider::HinaCollider(const SIM_DataFactory *factory) : SIM_Collider(factory) {}
HinaCollider::~HinaCollider() = default;

const SIM_DopDescription *HinaCollider::GetDescription()
{
	static PRM_Name theTest("test", "Test");

	static std::array<PRM_Template, 2> PRMS{
			PRM_Template(PRM_FLT_J,	1, &theTest, PRMoneDefaults),
			PRM_Template()
	};

	static SIM_DopDescription theDopDescription(true,
												"hina_collider",	// operator name
												"Hina Collider",	// English name
												"HinaCollider",	// default data name
												classname(),
												PRMS.data());

	return &theDopDescription;
}

bool HinaCollider::collideObjectsSubclass(SIM_Engine &engine, SIM_Object &object, SIM_Object &affector, const SIM_Time &starttime, const SIM_Time &endtime, SIM_Collider::SIM_ImpactApplyType impactapplytype, int impactflags) const
{
	return SIM_Collider::collideObjectsSubclass(engine, object, affector, starttime, endtime, impactapplytype, impactflags);
}
