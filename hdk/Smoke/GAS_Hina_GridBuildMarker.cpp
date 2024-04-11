#include "GAS_Hina_GridBuildMarker.h"

const char FLUID = 0;
const char AIR = 1;
const char BOUNDARY = 2;

GAS_HINA_SUBSOLVER_IMPLEMENT(
		GridBuildMarker,
		true,
		false,
		ACTIVATE_GAS_SOURCE \
        ACTIVATE_GAS_DENSITY \
        ACTIVATE_GAS_TEMPERATURE \
        ACTIVATE_GAS_COLLISION \
        ACTIVATE_GAS_VELOCITY \
        ACTIVATE_GAS_MARKER \
)

void GAS_Hina_GridBuildMarker::_init() {}
void GAS_Hina_GridBuildMarker::_makeEqual(const GAS_Hina_GridBuildMarker *src) {}
bool GAS_Hina_GridBuildMarker::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_ScalarField *S = getScalarField(obj, GAS_NAME_SOURCE);
	SIM_ScalarField *D = getScalarField(obj, GAS_NAME_DENSITY);
	SIM_ScalarField *T = getScalarField(obj, GAS_NAME_TEMPERATURE);
	SIM_ScalarField *C = getScalarField(obj, GAS_NAME_COLLISION);
	SIM_VectorField *V = getVectorField(obj, GAS_NAME_VELOCITY);
	SIM_ScalarField *M = getScalarField(obj, GAS_NAME_STENCIL);

	if (!S || !D || !T || !C || !V || !M)
		return false;

	if (!V->isFaceSampled())
		return false;

	_build(M->getField(), C->getField(), nullptr);

	return true;
}
void GAS_Hina_GridBuildMarker::_buildPartial(SIM_RawField *OutMarker, const SIM_RawField *BoundarySDF, const SIM_RawField *FluidSDF, const UT_JobInfo &info)
{
	UT_VoxelArrayIteratorF vit;
	OutMarker->getPartialRange(vit, info);
	vit.setCompressOnExit(true);
	for (vit.rewind(); !vit.atEnd(); vit.advance())
	{
		UT_Vector3 pos;
		OutMarker->indexToPos(vit.x(), vit.y(), vit.z(), pos);

		if (BoundarySDF->getValue(pos) < 0.0f)
			vit.setValue(BOUNDARY);
		else if (FluidSDF != nullptr && FluidSDF->getValue(pos) < 0.0f)
			vit.setValue(FLUID);
		else if (FluidSDF == nullptr)
			vit.setValue(FLUID);
		else
			vit.setValue(AIR);
	}
}
