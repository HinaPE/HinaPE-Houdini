#include "EmitterSolver.h"

void HinaPE::EmitterSolver::solve(SIM_RawField *IO_Field, SIM_RawField *IO_V_X, SIM_RawField *IO_V_Y, SIM_RawField *IO_V_Z)
{
	_emit(IO_Field, IO_V_X, IO_V_Y, IO_V_Z);
}
void HinaPE::EmitterSolver::_emitPartial(SIM_RawField *IO_Field, SIM_RawField *IO_V_X, SIM_RawField *IO_V_Y, SIM_RawField *IO_V_Z, const UT_JobInfo &info)
{
	UT_VoxelArrayIteratorF vit;
	IO_Field->getPartialRange(vit, info);
	vit.setCompressOnExit(true);
	for (vit.rewind(); !vit.atEnd(); vit.advance())
	{
		auto value = IO_Field->field()->getValue(vit.x(), vit.y(), vit.z());
		if (value > std::numeric_limits<fpreal32>::epsilon())
		{
			auto old_value = vit.getValue();
			vit.setValue(old_value + value);
		}
	}
}
