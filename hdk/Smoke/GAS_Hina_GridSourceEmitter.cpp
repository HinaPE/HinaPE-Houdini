#include "GAS_Hina_GridSourceEmitter.h"

GAS_HINA_SUBSOLVER_IMPLEMENT(
		GridSourceEmitter,
		true,
		false,
		HINA_BOOL_PARAMETER(EmitOnce, false) \
        ACTIVATE_GAS_SOURCE \
        ACTIVATE_GAS_DENSITY \
        ACTIVATE_GAS_TEMPERATURE \
        ACTIVATE_GAS_COLLISION \
        ACTIVATE_GAS_VELOCITY \
)

void GAS_Hina_GridSourceEmitter::_init()
{
	this->emitted = false;
}
void GAS_Hina_GridSourceEmitter::_makeEqual(const GAS_Hina_GridSourceEmitter *src)
{
	this->emitted = src->emitted;
}
bool GAS_Hina_GridSourceEmitter::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_ScalarField *S = getScalarField(obj, GAS_NAME_SOURCE);
	SIM_ScalarField *D = getScalarField(obj, GAS_NAME_DENSITY);
	SIM_ScalarField *T = getScalarField(obj, GAS_NAME_TEMPERATURE);
	SIM_ScalarField *C = getScalarField(obj, GAS_NAME_COLLISION);
	SIM_VectorField *V = getVectorField(obj, GAS_NAME_VELOCITY);

	if (!S || !D || !T || !C || !V)
		return false;

	if (!getEmitOnce() || !this->emitted)
	{
		_emit(D->getField(), S->getField());
		this->emitted = true;
	}

	return true;
}
void GAS_Hina_GridSourceEmitter::_emitPartial(SIM_RawField *Dest, const SIM_RawField *Src, const UT_JobInfo &info)
{
	UT_VoxelArrayIteratorF vit;
	Src->getPartialRange(vit, info);
	vit.setCompressOnExit(true);
	for (vit.rewind(); !vit.atEnd(); vit.advance())
	{
		auto value = Src->field()->getValue(vit.x(), vit.y(), vit.z());
		if (value > std::numeric_limits<fpreal32>::epsilon())
			Dest->fieldNC()->setValue(vit.x(), vit.y(), vit.z(), value);
	}
}
