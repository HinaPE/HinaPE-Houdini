#ifndef HINAPE_GAS_HINA_GRIDBUILDMARKER_H
#define HINAPE_GAS_HINA_GRIDBUILDMARKER_H

#include <SIM_Hina_Generator.h>

GAS_HINA_SUBSOLVER_CLASS(
		GridBuildMarker,

		THREADED_METHOD3(GAS_Hina_GridBuildMarker, OutMarker->shouldMultiThread(), _build, SIM_RawField *, OutMarker, const SIM_RawField *, BoundarySDF, const SIM_RawField *, FluidSDF);
		void _buildPartial(SIM_RawField *OutMarker, const SIM_RawField * BoundarySDF, const SIM_RawField * FluidSDF, const UT_JobInfo &info);

)

#endif //HINAPE_GAS_HINA_GRIDBUILDMARKER_H
