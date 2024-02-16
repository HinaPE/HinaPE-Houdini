#ifndef HINAPE_SIM_HINA_PARTICLESVISUALIZER_H
#define HINAPE_SIM_HINA_PARTICLESVISUALIZER_H

#include <SIM_Hina_Generator.h>

#include <iostream>

class SIM_Hina_ParticlesVisualizer : public SIM_Data, public SIM_OptionsUser
{
public:
	GET_GUIDE_FUNC_B(SIM_NAME_SHOWGUIDE, ShowGuideGeometry, true);
	GET_GUIDE_FUNC_I(SIM_NAME_OFFSET, InspectOffset, 0);

protected:
	explicit SIM_Hina_ParticlesVisualizer(const SIM_DataFactory *factory) : SIM_Data(factory), SIM_OptionsUser(this) {}
	~SIM_Hina_ParticlesVisualizer() override = default;
	SIM_Guide *createGuideObjectSubclass() const override;
	void buildGuideGeometrySubclass(const SIM_RootData &root, const SIM_Options &options, const GU_DetailHandle &gdh, UT_DMatrix4 *xform, const SIM_Time &t) const override;
	static const SIM_DopDescription	*getDopDescription();

DECLARE_STANDARD_GETCASTTOTYPE();
DECLARE_DATAFACTORY(SIM_Hina_ParticlesVisualizer,
					SIM_Data,
					"Hina Particles Visualizer",
					getDopDescription());
};

#endif //HINAPE_SIM_HINA_PARTICLESVISUALIZER_H
