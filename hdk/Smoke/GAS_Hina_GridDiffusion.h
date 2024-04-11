#ifndef HINAPE_GAS_HINA_GRIDDIFFUSION_H
#define HINAPE_GAS_HINA_GRIDDIFFUSION_H

#include <SIM_Hina_Generator.h>

GAS_HINA_SUBSOLVER_CLASS(
		GridDiffusion,

		THREADED_METHOD6(GAS_Hina_GridDiffusion, OutField->shouldMultiThread(), _diffusion, float, dt, float, coefficient, SIM_RawField * , OutField, const SIM_RawField *, InField, const SIM_RawField *, BoundarySDF, const SIM_RawField *, FluidSDF);
		void _diffusionPartial(float dt, float coefficient, SIM_RawField *OutField, const SIM_RawField *InField, const SIM_RawField *BoundarySDF, const SIM_RawField *FluidSDF, const UT_JobInfo &info);

		THREADED_METHOD4(GAS_Hina_GridDiffusion, OutMarkers->shouldMultiThread(), _build_markers, SIM_RawField * , OutMarkers, const SIM_RawField *, InField, const SIM_RawField *, BoundarySDF, const SIM_RawField *, FluidSDF);
		void _build_markersPartial(SIM_RawField *OutMarkers, const SIM_RawField *InField, const SIM_RawField *BoundarySDF, const SIM_RawField *FluidSDF, const UT_JobInfo &info);


)

#endif //HINAPE_GAS_HINA_GRIDDIFFUSION_H
