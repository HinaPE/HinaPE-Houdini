#include "FieldUtils.h"

CubbyFlow::ScalarGrid3Ptr HinaPE::ToCubby(const SIM_RawField &Field)
{
	CubbyFlow::ScalarGrid3Ptr Output;
	CubbyFlow::Vector3UZ resolution = {(size_t) Field.getVoxelRes().x(), (size_t) Field.getVoxelRes().y(), (size_t) Field.getVoxelRes().z()};
	CubbyFlow::Vector3D spacing = {Field.getVoxelSize().x(), Field.getVoxelSize().y(), Field.getVoxelSize().z()};
	CubbyFlow::Vector3D origin = {Field.getOrig().x(), Field.getOrig().y(), Field.getOrig().z()};

	switch (Field.getSample())
	{
		case SIM_SAMPLE_CENTER:
		{
			Output = CubbyFlow::CellCenteredScalarGrid3::GetBuilder().MakeShared();
		}
			break;
		case SIM_SAMPLE_CORNER:
		{
			Output = CubbyFlow::VertexCenteredScalarGrid3::GetBuilder().MakeShared();
		}
			break;
		default:
			break;
	}

	Output->Resize(resolution, spacing, origin);
	Output->ParallelForEachDataPointIndex(
			[&](const CubbyFlow::Vector3UZ &idx)
			{
				(*Output)(idx.x, idx.y, idx.z) = Field.field()->getValue(idx.x, idx.y, idx.z);
			});
	return Output;
}
SIM_RawField HinaPE::ToHDK(const CubbyFlow::ScalarGrid3Ptr &Field)
{
	SIM_RawField Output;
	UT_Vector3 origin = {(real) Field->Origin().x, (real) Field->Origin().y, (real) Field->Origin().z}; // TODO: Origin or DataOrigin?
	UT_Vector3 size = {
			(real) Field->GridSpacing().x * Field->Resolution().x,
			(real) Field->GridSpacing().y * Field->Resolution().y,
			(real) Field->GridSpacing().z * Field->Resolution().z
	};
	UT_Vector3I resolution = {(int) Field->DataSize().x, (int) Field->DataSize().y, (int) Field->DataSize().z};
	if (std::dynamic_pointer_cast<CubbyFlow::CellCenteredScalarGrid3>(Field))
	{
		Output.init(SIM_SAMPLE_CENTER, origin, size, resolution.x(), resolution.y(), resolution.z());
	} else if (std::dynamic_pointer_cast<CubbyFlow::VertexCenteredScalarGrid3>(Field))
	{
		Output.init(SIM_SAMPLE_CORNER, origin, size, resolution.x(), resolution.y(), resolution.z());
	}
	return Output;
}
HinaPE::BackwardDiffusionSolver::BackwardDiffusionSolver()
{
	solver = std::make_shared<CubbyFlow::GridBackwardEulerDiffusionSolver3>();
}
void HinaPE::BackwardDiffusionSolver::Solve(const SIM_RawField &Input, SIM_RawField &Marker, SIM_RawField &Output)
{
}
