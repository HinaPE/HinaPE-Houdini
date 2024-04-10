#include "GAS_Hina_GridBoundarySolver.h"

enum GridType
{
	BOUNDARY = 0,
	FLUID = 1,
};

GAS_HINA_SUBSOLVER_IMPLEMENT(
		GridBoundarySolver,
		true,
		false,
		HINA_INT_PARAMETER(Iter, 4) \
        ACTIVATE_GAS_SOURCE \
        ACTIVATE_GAS_DENSITY \
        ACTIVATE_GAS_TEMPERATURE \
        ACTIVATE_GAS_COLLISION \
        ACTIVATE_GAS_VELOCITY \
)

void GAS_Hina_GridBoundarySolver::_init() {}
void GAS_Hina_GridBoundarySolver::_makeEqual(const GAS_Hina_GridBoundarySolver *src) {}
bool GAS_Hina_GridBoundarySolver::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	return true;
}
void GAS_Hina_GridBoundarySolver::_extrapolatePartial(SIM_RawField *Field, SIM_RawField *Valid, const UT_JobInfo &info)
{
	UT_VoxelArrayIteratorF vit;
	Field->getPartialRange(vit, info);
	vit.setCompressOnExit(true);
	for (vit.rewind(); !vit.atEnd(); vit.advance())
	{
		if (Valid->field()->getValue(vit.x(), vit.y(), vit.z()) == GridType::BOUNDARY)
		{
			fpreal32 sum = 0.0f;
			size_t count = 0;

			if (vit.x() + 1 < Field->field()->getXRes() && Valid->field()->getValue(vit.x() + 1, vit.y(), vit.z()) == GridType::FLUID)
			{
				sum += Field->field()->getValue(vit.x() + 1, vit.y(), vit.z());
				++count;
			}

			if (vit.x() > 0 && Valid->field()->getValue(vit.x() - 1, vit.y(), vit.z()) == GridType::FLUID)
			{
				sum += Field->field()->getValue(vit.x() - 1, vit.y(), vit.z());
				++count;
			}

			if (vit.y() + 1 < Field->field()->getYRes() && Valid->field()->getValue(vit.x(), vit.y() + 1, vit.z()) == GridType::FLUID)
			{
				sum += Field->field()->getValue(vit.x(), vit.y() + 1, vit.z());
				++count;
			}

			if (vit.y() > 0 && Valid->field()->getValue(vit.x(), vit.y() - 1, vit.z()) == GridType::FLUID)
			{
				sum += Field->field()->getValue(vit.x(), vit.y() - 1, vit.z());
				++count;
			}

			if (vit.z() + 1 < Field->field()->getZRes() && Valid->field()->getValue(vit.x(), vit.y(), vit.z() + 1) == GridType::FLUID)
			{
				sum += Field->field()->getValue(vit.x(), vit.y(), vit.z() + 1);
				++count;
			}

			if (vit.z() > 0 && Valid->field()->getValue(vit.x(), vit.y(), vit.z() - 1) == GridType::FLUID)
			{
				sum += Field->field()->getValue(vit.x(), vit.y(), vit.z() - 1);
				++count;
			}

			if (count > 0)
			{
				Field->fieldNC()->setValue(vit.x(), vit.y(), vit.z(), (sum / count));
				Valid->fieldNC()->setValue(vit.x(), vit.y(), vit.z(), GridType::FLUID);
			}
		}
	}
}
