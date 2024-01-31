#ifndef HINAPE_SIM_HINA_PARTICLES_H
#define HINAPE_SIM_HINA_PARTICLES_H

#include <Common/SIM_Hina_Generator.h>

SIM_HINA_GEOMETRY_CLASS(
		Particles,
		HINA_GETSET_PARAMETER(FluidDomain, GETSET_DATA_FUNCS_V3)
		HINA_GETSET_PARAMETER(TargetSpacing, GETSET_DATA_FUNCS_F)
		HINA_GETSET_PARAMETER(KernelRadiusOverTargetSpacing, GETSET_DATA_FUNCS_F)
		HINA_GETSET_PARAMETER(TargetDensity, GETSET_DATA_FUNCS_F)

		virtual void Commit();
		mutable fpreal Mass;
		std::map<GA_Offset, std::vector<GA_Offset>> neighbor_lists_cache;
		std::map<GA_Offset, UT_Vector3> positions_cache;
		std::map<GA_Offset, UT_Vector3> velocity_cache;
)

GAS_HINA_SUBSOLVER_CLASS(
		CommitCache,

		SIM_Guide *createGuideObjectSubclass() const override;
		void buildGuideGeometrySubclass(const SIM_RootData &root, const SIM_Options &options, const GU_DetailHandle &gdh, UT_DMatrix4 *xform, const SIM_Time &t) const override;
)

#endif //HINAPE_SIM_HINA_PARTICLES_H
