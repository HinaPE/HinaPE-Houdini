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
			origin += spacing * 0.5;
			Output = CubbyFlow::VertexCenteredScalarGrid3::GetBuilder().MakeShared();
		}
			break;
		case SIM_SAMPLE_FACEX:
		{

		}
			break;
		case SIM_SAMPLE_FACEY:
		{

		}
			break;
		case SIM_SAMPLE_FACEZ:
		{

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
			(real) Field->GridSpacing().x * Field->DataSize().x,
			(real) Field->GridSpacing().y * Field->DataSize().y,
			(real) Field->GridSpacing().z * Field->DataSize().z
	};
	UT_Vector3I resolution = {(int) Field->Resolution().x, (int) Field->Resolution().y, (int) Field->Resolution().z};
	if (std::dynamic_pointer_cast<CubbyFlow::CellCenteredScalarGrid3>(Field))
	{
		Output.init(SIM_SAMPLE_CENTER, origin, size, resolution.x(), resolution.y(), resolution.z());
	} else if (std::dynamic_pointer_cast<CubbyFlow::VertexCenteredScalarGrid3>(Field))
	{
		size -= UT_Vector3D{Field->GridSpacing().x, Field->GridSpacing().y, Field->GridSpacing().z};
		Output.init(SIM_SAMPLE_CORNER, origin, size, resolution.x(), resolution.y(), resolution.z());
	}
	return Output;
}
bool HinaPE::match(const SIM_RawField &Field1, const CubbyFlow::ScalarGrid3 &Field2)
{
	Field2.ForEachDataPointIndex(
			[&](const CubbyFlow::Vector3UZ &idx)
			{
				UT_Vector3 pos1 = UT_Vector3D{Field2.DataPosition()(idx.x, idx.y, idx.z).x, Field2.DataPosition()(idx.x, idx.y, idx.z).y, Field2.DataPosition()(idx.x, idx.y, idx.z).z};
				UT_Vector3 pos2 = Field1.indexToPos(UT_Vector3I{(int) idx.x, (int) idx.y, (int) idx.z});

				if (pos1.distance(pos2) > 1e-6)
				{
					std::cout << "Mismatch: " << pos1 << " != " << pos2 << std::endl;
					return false;
				}
			});
	return true;
}
void HinaPE::print(const SIM_RawField &Field)
{
	std::cout << "HDK Origin: " << Field.getOrig() << std::endl;
	std::cout << "HDK Resolution: : " << Field.getVoxelRes() << std::endl;
	std::cout << "HDK VoxelSize: : " << Field.getVoxelSize() << std::endl;
	std::cout << "HDK Size: " << Field.getSize() << std::endl;
	std::cout << "HDK Data0 Pos: " << Field.indexToPos({0, 0, 0}) << std::endl;
}
void HinaPE::print(const CubbyFlow::ScalarGrid3 &Field)
{
	std::cout << "Cubby Origin: " << Field.Origin().x << ", " << Field.Origin().y << ", " << Field.Origin().z << std::endl;
	std::cout << "Cubby Resolution: : " << Field.Resolution().x << ", " << Field.Resolution().y << ", " << Field.Resolution().z << std::endl;
	std::cout << "Cubby VoxelSize: : " << Field.GridSpacing().x << ", " << Field.GridSpacing().y << ", " << Field.GridSpacing().z << std::endl;
	std::cout << "Cubby Size: " << Field.DataSize().x * Field.GridSpacing().x << ", " << Field.DataSize().y * Field.GridSpacing().y << ", " << Field.DataSize().z * Field.GridSpacing().z << std::endl;
	std::cout << "Cubby Data0 Pos: " << Field.DataPosition()(0, 0, 0).x << ", " << Field.DataPosition()(0, 0, 0).y << ", " << Field.DataPosition()(0, 0, 0).z << std::endl;
}

HinaPE::BackwardDiffusionSolver::BackwardDiffusionSolver()
{
	solver = std::make_shared<CubbyFlow::GridBackwardEulerDiffusionSolver3>();
}
void HinaPE::BackwardDiffusionSolver::Solve(const SIM_RawField &Input, SIM_RawField &Marker, SIM_RawField &Output)
{
}
