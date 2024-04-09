#include "FieldUtils.h"

void HinaPE::ToCubby(const SIM_ScalarField &Field, CubbyFlow::ScalarGrid3Ptr &Output)
{
	CubbyFlow::Vector3UZ resolution = {(size_t) Field.getDivisions().x(), (size_t) Field.getDivisions().y(), (size_t) Field.getDivisions().z()};
	CubbyFlow::Vector3D spacing = {Field.getVoxelSize().x(), Field.getVoxelSize().y(), Field.getVoxelSize().z()};
	CubbyFlow::Vector3D origin = {Field.getOrig().x(), Field.getOrig().y(), Field.getOrig().z()};

	if (Field.getVoxelSample() == SIM_SAMPLE_CENTER)
		Output = CubbyFlow::CellCenteredScalarGrid3::GetBuilder().MakeShared();
	else if (Field.getVoxelSample() == SIM_SAMPLE_CORNER)
		Output = CubbyFlow::VertexCenteredScalarGrid3::GetBuilder().MakeShared();
	else {}

	Output->Resize(resolution, spacing, origin);
	Output->ParallelForEachDataPointIndex(
			[&](const CubbyFlow::Vector3UZ &idx)
			{
				(*Output)(idx.x, idx.y, idx.z) = Field.getField()->field()->getValue(idx.x, idx.y, idx.z);
			});
}
void HinaPE::ToCubby(const SIM_VectorField &Field, CubbyFlow::VectorGrid3Ptr &Output)
{
	CubbyFlow::Vector3UZ resolution = {(size_t) Field.getDivisions().x(), (size_t) Field.getDivisions().y(), (size_t) Field.getDivisions().z()};
	CubbyFlow::Vector3D spacing = {Field.getVoxelSize().x(), Field.getVoxelSize().y(), Field.getVoxelSize().z()};
	CubbyFlow::Vector3D origin = {Field.getOrig().x(), Field.getOrig().y(), Field.getOrig().z()};

	if (Field.isCenterSampled())
	{
		CubbyFlow::CellCenteredVectorGrid3Ptr _ = CubbyFlow::CellCenteredVectorGrid3::GetBuilder().MakeShared();
		_->Resize(resolution, spacing, origin);
		_->ParallelForEachDataPointIndex(
				[&](const CubbyFlow::Vector3UZ &idx)
				{
					(*_)(idx.x, idx.y, idx.z) =
							{Field.getXField()->field()->getValue(idx.x, idx.y, idx.z),
							 Field.getYField()->field()->getValue(idx.x, idx.y, idx.z),
							 Field.getZField()->field()->getValue(idx.x, idx.y, idx.z)};
				});
		Output = _;
	} else if (Field.isCornerSampled())
	{
		CubbyFlow::VertexCenteredVectorGrid3Ptr _ = CubbyFlow::VertexCenteredVectorGrid3::GetBuilder().MakeShared();
		_->Resize(resolution, spacing, origin);
		_->ParallelForEachDataPointIndex(
				[&](const CubbyFlow::Vector3UZ &idx)
				{
					(*_)(idx.x, idx.y, idx.z) =
							{Field.getXField()->field()->getValue(idx.x, idx.y, idx.z),
							 Field.getYField()->field()->getValue(idx.x, idx.y, idx.z),
							 Field.getZField()->field()->getValue(idx.x, idx.y, idx.z)};
				});
		Output = _;
	} else if (Field.isFaceSampled())
	{
		CubbyFlow::FaceCenteredGrid3Ptr _ = CubbyFlow::FaceCenteredGrid3::GetBuilder().MakeShared();
		_->Resize(resolution, spacing, origin);
		_->ParallelForEachUIndex(
				[&](const CubbyFlow::Vector3UZ &idx)
				{
					_->U(idx) = Field.getXField()->field()->getValue(idx.x, idx.y, idx.z);
					_->V(idx) = Field.getYField()->field()->getValue(idx.x, idx.y, idx.z);
					_->W(idx) = Field.getZField()->field()->getValue(idx.x, idx.y, idx.z);
				});
		Output = _;
	} else {}
}
void HinaPE::ToHDK(const CubbyFlow::ScalarGrid3Ptr &Field, SIM_ScalarField &Output)
{
//	UT_Vector3 origin = {(real) Field->Origin().x, (real) Field->Origin().y, (real) Field->Origin().z}; // TODO: Origin or DataOrigin?
//	UT_Vector3 size = {
//			(real) Field->GridSpacing().x * Field->DataSize().x,
//			(real) Field->GridSpacing().y * Field->DataSize().y,
//			(real) Field->GridSpacing().z * Field->DataSize().z
//	};
//	UT_Vector3I resolution = {(int) Field->Resolution().x, (int) Field->Resolution().y, (int) Field->Resolution().z};
}
void HinaPE::ToHDK(const CubbyFlow::VectorGrid3Ptr &Field, SIM_VectorField &Output)
{

}
void HinaPE::match(const SIM_ScalarField &FieldHDK, const CubbyFlow::CellCenteredScalarGrid3Ptr &FieldCubby)
{
	FieldCubby->ForEachDataPointIndex(
			[&](const CubbyFlow::Vector3UZ &idx)
			{
				UT_Vector3 pos1 = UT_Vector3D{FieldCubby->DataPosition()(idx.x, idx.y, idx.z).x, FieldCubby->DataPosition()(idx.x, idx.y, idx.z).y, FieldCubby->DataPosition()(idx.x, idx.y, idx.z).z};
				UT_Vector3 pos2;
				FieldHDK.indexToPos((int) idx.x, (int) idx.y, (int) idx.z, pos2);

				if (pos1.distance(pos2) > 1e-6) { std::cout << "Mismatch: " << pos1 << " != " << pos2 << std::endl; }
			});
}
void HinaPE::match(const SIM_VectorField &FieldHDK, const CubbyFlow::FaceCenteredGrid3Ptr &FieldCubby)
{
	FieldCubby->ForEachUIndex(
			[&](const CubbyFlow::Vector3UZ &idx)
			{
				int AXIS_X = 0, AXIS_Y = 1, AXIS_Z = 2;

				{
					UT_Vector3 pos1 = UT_Vector3D{FieldCubby->DataPosition(AXIS_X)(idx.x, idx.y, idx.z).x, FieldCubby->DataPosition(AXIS_X)(idx.x, idx.y, idx.z).y, FieldCubby->DataPosition(AXIS_X)(idx.x, idx.y, idx.z).z};
					UT_Vector3 pos2;
					FieldHDK.indexToPos(AXIS_X, (int) idx.x, (int) idx.y, (int) idx.z, pos2);
					if (pos1.distance(pos2) > 1e-6) { std::cout << "Mismatch: " << pos1 << " != " << pos2 << std::endl; }
				}

				{
					UT_Vector3 pos1 = UT_Vector3D{FieldCubby->DataPosition(AXIS_Y)(idx.x, idx.y, idx.z).x, FieldCubby->DataPosition(AXIS_Y)(idx.x, idx.y, idx.z).y, FieldCubby->DataPosition(AXIS_Y)(idx.x, idx.y, idx.z).z};
					UT_Vector3 pos2;
					FieldHDK.indexToPos(AXIS_Y, (int) idx.x, (int) idx.y, (int) idx.z, pos2);
					if (pos1.distance(pos2) > 1e-6) { std::cout << "Mismatch: " << pos1 << " != " << pos2 << std::endl; }
				}

				{
					UT_Vector3 pos1 = UT_Vector3D{FieldCubby->DataPosition(AXIS_Z)(idx.x, idx.y, idx.z).x, FieldCubby->DataPosition(AXIS_Z)(idx.x, idx.y, idx.z).y, FieldCubby->DataPosition(AXIS_Z)(idx.x, idx.y, idx.z).z};
					UT_Vector3 pos2;
					FieldHDK.indexToPos(AXIS_Z, (int) idx.x, (int) idx.y, (int) idx.z, pos2);
					if (pos1.distance(pos2) > 1e-6) { std::cout << "Mismatch: " << pos1 << " != " << pos2 << std::endl; }
				}
			});
}
void HinaPE::print(const SIM_ScalarField &Field)
{
	std::cout << "SIM_ScalarField Size: " << Field.getSize() << std::endl;
	std::cout << "SIM_ScalarField Center: " << Field.getCenter() << std::endl;
	std::cout << "SIM_ScalarField Divisions: " << Field.getDivisions() << std::endl;
	std::cout << "SIM_ScalarField Origin: " << Field.getOrig() << std::endl;
}
void HinaPE::print(const SIM_VectorField &Field)
{
	std::cout << "SIM_VectorField Size: " << Field.getSize() << std::endl;
	std::cout << "SIM_VectorField Center: " << Field.getCenter() << std::endl;
	std::cout << "SIM_VectorField Divisions: " << Field.getDivisions() << std::endl;
	std::cout << "SIM_VectorField Origin: " << Field.getOrig() << std::endl;
}
void HinaPE::print(const SIM_RawField &Field)
{
	std::cout << "SIM_RawField Origin: " << Field.getOrig() << std::endl;
	std::cout << "SIM_RawField Resolution: : " << Field.getVoxelRes() << std::endl;
	std::cout << "SIM_RawField VoxelSize: : " << Field.getVoxelSize() << std::endl;
	std::cout << "SIM_RawField Size: " << Field.getSize() << std::endl;
	std::cout << "SIM_RawField Data0 Pos: " << Field.indexToPos({0, 0, 0}) << std::endl;
}

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
