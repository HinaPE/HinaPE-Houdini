#include "EmitterSolver.h"

void HinaPE::EmitterSolver::_emitPartial(SIM_RawField *OutField, const SIM_RawField *InField, SIM_RawField *OutFlow_X, SIM_RawField *OutFlow_Y, SIM_RawField *OutFlow_Z, const UT_JobInfo &info)
{
	UT_VoxelArrayIteratorF vit;
	OutField->getPartialRange(vit, info);
	vit.setCompressOnExit(true);
	for (vit.rewind(); !vit.atEnd(); vit.advance())
	{
		UT_Vector3 pos;
		OutField->indexToPos(vit.x(), vit.y(), vit.z(), pos);
		auto value = InField->getValue(pos);
		if (value > std::numeric_limits<fpreal32>::epsilon())
		{
			auto old_value = vit.getValue();
			vit.setValue(old_value + value);
		}
	}
}
