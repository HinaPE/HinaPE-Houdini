#ifndef HINAPE_HOUDINI_COLLIDER_H
#define HINAPE_HOUDINI_COLLIDER_H

#include <SIM/SIM_Collider.h>
#include <SIM/SIM_DopDescription.h>

class HinaCollider : public SIM_Collider
{
protected:
	explicit HinaCollider(const SIM_DataFactory *factory);
	~HinaCollider() override;
	bool collideObjectsSubclass(SIM_Engine &engine, SIM_Object &object, SIM_Object &affector, const SIM_Time &starttime, const SIM_Time &endtime, SIM_ImpactApplyType impactapplytype, int impactflags) const override;

private:
	static const SIM_DopDescription* GetDescription();
DECLARE_STANDARD_GETCASTTOTYPE();
DECLARE_DATAFACTORY(HinaCollider, SIM_Collider, "Hina Collider", GetDescription());
};

#endif //HINAPE_HOUDINI_COLLIDER_H
