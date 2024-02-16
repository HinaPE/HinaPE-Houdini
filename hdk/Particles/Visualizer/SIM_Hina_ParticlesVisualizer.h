#ifndef HINAPE_SIM_HINA_PARTICLESVISUALIZER_H
#define HINAPE_SIM_HINA_PARTICLESVISUALIZER_H

#include <SIM_Hina_Generator.h>

class SIM_Hina_ParticlesNeighborsVisualizer : public SIM_Data, public SIM_OptionsUser
{
public:
	GET_GUIDE_FUNC_B(SIM_NAME_SHOWGUIDE, ShowGuideGeometry, true);
	GET_GUIDE_FUNC_I(SIM_NAME_OFFSET, InspectOffset, 0);
	GET_GUIDE_FUNC_V3("DomainColor", DomainColor, (.0156356, 0, .5));

protected:
	explicit SIM_Hina_ParticlesNeighborsVisualizer(const SIM_DataFactory *factory) : SIM_Data(factory), SIM_OptionsUser(this) {}
	~SIM_Hina_ParticlesNeighborsVisualizer() override = default;
	SIM_Guide *createGuideObjectSubclass() const override;
	void buildGuideGeometrySubclass(const SIM_RootData &root, const SIM_Options &options, const GU_DetailHandle &gdh, UT_DMatrix4 *xform, const SIM_Time &t) const override;
	static const SIM_DopDescription	*getDopDescription();

DECLARE_STANDARD_GETCASTTOTYPE();
DECLARE_DATAFACTORY(SIM_Hina_ParticlesNeighborsVisualizer,
					SIM_Data,
					"Hina Particles Neighbors Visualizer",
					getDopDescription());
};

#endif //HINAPE_SIM_HINA_PARTICLESVISUALIZER_H
