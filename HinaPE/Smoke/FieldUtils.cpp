#include "FieldUtils.h"

void HinaPE::ToCubby(const SIM_ScalarField &Field, CubbyFlow::CellCenteredScalarGrid3Ptr &Output)
{
	CubbyFlow::Vector3UZ resolution = {(size_t) Field.getDivisions().x(), (size_t) Field.getDivisions().y(), (size_t) Field.getDivisions().z()};
	CubbyFlow::Vector3D spacing = {Field.getVoxelSize().x(), Field.getVoxelSize().y(), Field.getVoxelSize().z()};
	CubbyFlow::Vector3D origin = {Field.getOrig().x(), Field.getOrig().y(), Field.getOrig().z()};

	if (Field.getVoxelSample() == SIM_SAMPLE_CENTER)
		Output = CubbyFlow::CellCenteredScalarGrid3::GetBuilder().MakeShared();
	else
		std::cout << "ERROR VOXEL SAMPLE MATCHED" << std::endl;

	Output->Resize(resolution, spacing, origin);
	Output->ParallelForEachDataPointIndex(
			[&](const CubbyFlow::Vector3UZ &idx)
			{
				(*Output)(idx.x, idx.y, idx.z) = Field.getField()->field()->getValue(idx.x, idx.y, idx.z);
			});
}
void HinaPE::ToCubby(const SIM_ScalarField &Field, CubbyFlow::VertexCenteredScalarGrid3Ptr &Output)
{
	CubbyFlow::Vector3UZ resolution = {(size_t) Field.getDivisions().x(), (size_t) Field.getDivisions().y(), (size_t) Field.getDivisions().z()};
	CubbyFlow::Vector3D spacing = {Field.getVoxelSize().x(), Field.getVoxelSize().y(), Field.getVoxelSize().z()};
	CubbyFlow::Vector3D origin = {Field.getOrig().x(), Field.getOrig().y(), Field.getOrig().z()};

	if (Field.getVoxelSample() == SIM_SAMPLE_CORNER)
		Output = CubbyFlow::VertexCenteredScalarGrid3::GetBuilder().MakeShared();
	else
		std::cout << "ERROR VOXEL SAMPLE MATCHED" << std::endl;

	Output->Resize(resolution, spacing, origin);
	Output->ParallelForEachDataPointIndex(
			[&](const CubbyFlow::Vector3UZ &idx)
			{
				(*Output)(idx.x, idx.y, idx.z) = Field.getField()->field()->getValue(idx.x, idx.y, idx.z);
			});
}
void HinaPE::ToCubby(const SIM_VectorField &Field, CubbyFlow::CellCenteredVectorGrid3Ptr &Output)
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
	} else
		std::cout << "ERROR VOXEL SAMPLE MATCHED" << std::endl;
}
void HinaPE::ToCubby(const SIM_VectorField &Field, CubbyFlow::VertexCenteredVectorGrid3Ptr &Output)
{
	CubbyFlow::Vector3UZ resolution = {(size_t) Field.getDivisions().x(), (size_t) Field.getDivisions().y(), (size_t) Field.getDivisions().z()};
	CubbyFlow::Vector3D spacing = {Field.getVoxelSize().x(), Field.getVoxelSize().y(), Field.getVoxelSize().z()};
	CubbyFlow::Vector3D origin = {Field.getOrig().x(), Field.getOrig().y(), Field.getOrig().z()};

	if (Field.isCornerSampled())
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
	} else
		std::cout << "ERROR VOXEL SAMPLE MATCHED" << std::endl;
}
void HinaPE::ToCubby(const SIM_VectorField &Field, CubbyFlow::FaceCenteredGrid3Ptr &Output)
{
	CubbyFlow::Vector3UZ resolution = {(size_t) Field.getDivisions().x(), (size_t) Field.getDivisions().y(), (size_t) Field.getDivisions().z()};
	CubbyFlow::Vector3D spacing = {Field.getVoxelSize().x(), Field.getVoxelSize().y(), Field.getVoxelSize().z()};
	CubbyFlow::Vector3D origin = {Field.getOrig().x(), Field.getOrig().y(), Field.getOrig().z()};

	if (Field.isFaceSampled())
	{
		CubbyFlow::FaceCenteredGrid3Ptr _ = CubbyFlow::FaceCenteredGrid3::GetBuilder().MakeShared();
		_->Resize(resolution, spacing, origin);
		_->ParallelForEachUIndex(
				[&](const CubbyFlow::Vector3UZ &idx)
				{
					_->U(idx) = Field.getXField()->field()->getValue(idx.x, idx.y, idx.z);
				});
		_->ParallelForEachVIndex(
				[&](const CubbyFlow::Vector3UZ &idx)
				{
					_->V(idx) = Field.getYField()->field()->getValue(idx.x, idx.y, idx.z);
				});
		_->ParallelForEachWIndex(
				[&](const CubbyFlow::Vector3UZ &idx)
				{
					_->W(idx) = Field.getZField()->field()->getValue(idx.x, idx.y, idx.z);
				});
		Output = _;
	} else
		std::cout << "ERROR VOXEL SAMPLE MATCHED" << std::endl;
}

void HinaPE::ToHDK(const CubbyFlow::ScalarGrid3Ptr &Field, SIM_ScalarField &Output)
{
	UT_Vector3 origin = {(real) Field->Origin().x, (real) Field->Origin().y, (real) Field->Origin().z};
	UT_Vector3 size = {
			(real) Field->GridSpacing().x * Field->Resolution().x,
			(real) Field->GridSpacing().y * Field->Resolution().y,
			(real) Field->GridSpacing().z * Field->Resolution().z
	};
	UT_Vector3I resolution = {(int) Field->Resolution().x, (int) Field->Resolution().y, (int) Field->Resolution().z};
	if (std::dynamic_pointer_cast<CubbyFlow::CellCenteredScalarGrid3>(Field))
	{
		Output.getField()->init(SIM_SAMPLE_CENTER, origin, size, resolution.x(), resolution.y(), resolution.z());
	} else if (std::dynamic_pointer_cast<CubbyFlow::VertexCenteredScalarGrid3>(Field))
	{
		Output.getField()->init(SIM_SAMPLE_CORNER, origin, size, resolution.x(), resolution.y(), resolution.z());
	}
}
void HinaPE::ToHDK(const CubbyFlow::VectorGrid3Ptr &Field, SIM_VectorField &Output)
{
	UT_Vector3 origin = {(real) Field->Origin().x, (real) Field->Origin().y, (real) Field->Origin().z};
	UT_Vector3 size = {
			(real) Field->GridSpacing().x * Field->Resolution().x,
			(real) Field->GridSpacing().y * Field->Resolution().y,
			(real) Field->GridSpacing().z * Field->Resolution().z
	};
	UT_Vector3I resolution = {(int) Field->Resolution().x, (int) Field->Resolution().y, (int) Field->Resolution().z};
	if (std::dynamic_pointer_cast<CubbyFlow::FaceCenteredGrid3>(Field))
	{
		Output.getXField()->init(SIM_SAMPLE_FACEX, origin, size, resolution.x(), resolution.y(), resolution.z());
		Output.getYField()->init(SIM_SAMPLE_FACEY, origin, size, resolution.x(), resolution.y(), resolution.z());
		Output.getZField()->init(SIM_SAMPLE_FACEZ, origin, size, resolution.x(), resolution.y(), resolution.z());
	}
}
void HinaPE::match(const SIM_ScalarField &FieldHDK, const CubbyFlow::CellCenteredScalarGrid3Ptr &FieldCubby)
{
	FieldCubby->ForEachDataPointIndex(
			[&](const CubbyFlow::Vector3UZ &idx)
			{
				UT_Vector3 pos1 = UT_Vector3D{FieldCubby->DataPosition()(idx.x, idx.y, idx.z).x, FieldCubby->DataPosition()(idx.x, idx.y, idx.z).y, FieldCubby->DataPosition()(idx.x, idx.y, idx.z).z};
				UT_Vector3 pos2;
				FieldHDK.indexToPos((int) idx.x, (int) idx.y, (int) idx.z, pos2);

				if (pos1.distance(pos2) < 1e-6) { std::cout << "Mismatch: " << pos1 << " != " << pos2 << std::endl; }
			});
	std::cout << "Match! " << std::endl;
}
void HinaPE::match(const SIM_ScalarField &FieldHDK, const CubbyFlow::VertexCenteredScalarGrid3Ptr &FieldCubby)
{
	FieldCubby->ForEachDataPointIndex(
			[&](const CubbyFlow::Vector3UZ &idx)
			{
				UT_Vector3 pos1 = UT_Vector3D{FieldCubby->DataPosition()(idx.x, idx.y, idx.z).x, FieldCubby->DataPosition()(idx.x, idx.y, idx.z).y, FieldCubby->DataPosition()(idx.x, idx.y, idx.z).z};
				UT_Vector3 pos2;
				FieldHDK.indexToPos((int) idx.x, (int) idx.y, (int) idx.z, pos2);

				if (pos1.distance(pos2) > 1e-6) { std::cout << "Mismatch: " << pos1 << " != " << pos2 << std::endl; }
			});
	std::cout << "Match! " << std::endl;
}
void HinaPE::match(const SIM_VectorField &FieldHDK, const CubbyFlow::CellCenteredVectorGrid3Ptr &FieldCubby)
{
	FieldCubby->ForEachDataPointIndex(
			[&](const CubbyFlow::Vector3UZ &idx)
			{
				UT_Vector3 pos1 = UT_Vector3D{FieldCubby->DataPosition()(idx.x, idx.y, idx.z).x, FieldCubby->DataPosition()(idx.x, idx.y, idx.z).y, FieldCubby->DataPosition()(idx.x, idx.y, idx.z).z};
				UT_Vector3 pos2;
				FieldHDK.indexToPos(0, (int) idx.x, (int) idx.y, (int) idx.z, pos2);

				if (pos1.distance(pos2) > 1e-6) { std::cout << "Mismatch: " << pos1 << " != " << pos2 << std::endl; }
			});
	std::cout << "Match! " << std::endl;
}
void HinaPE::match(const SIM_VectorField &FieldHDK, const CubbyFlow::VertexCenteredVectorGrid3Ptr &FieldCubby)
{
	FieldCubby->ForEachDataPointIndex(
			[&](const CubbyFlow::Vector3UZ &idx)
			{
				UT_Vector3 pos1 = UT_Vector3D{FieldCubby->DataPosition()(idx.x, idx.y, idx.z).x, FieldCubby->DataPosition()(idx.x, idx.y, idx.z).y, FieldCubby->DataPosition()(idx.x, idx.y, idx.z).z};
				UT_Vector3 pos2;
				FieldHDK.indexToPos(0, (int) idx.x, (int) idx.y, (int) idx.z, pos2);

				if (pos1.distance(pos2) > 1e-6) { std::cout << "Mismatch: " << pos1 << " != " << pos2 << std::endl; }
			});
	std::cout << "Match! " << std::endl;
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
	std::cout << "Match! " << std::endl;
}
void HinaPE::print(const SIM_ScalarField &Field)
{
	std::cout << "SIM_ScalarField Size: " << Field.getSize() << std::endl;
	std::cout << "SIM_ScalarField Center: " << Field.getCenter() << std::endl;
	std::cout << "SIM_ScalarField Divisions: " << Field.getDivisions() << std::endl;
	std::cout << "SIM_ScalarField Origin: " << Field.getOrig() << std::endl;
	std::cout << "SIM_ScalarField Data0: " << Field.getField()->indexToPos({0, 0, 0}) << std::endl;
}
void HinaPE::print(const SIM_VectorField &Field)
{
	std::cout << "SIM_VectorField Size: " << Field.getSize() << std::endl;
	std::cout << "SIM_VectorField Center: " << Field.getCenter() << std::endl;
	std::cout << "SIM_VectorField Divisions: " << Field.getDivisions() << std::endl;
	std::cout << "SIM_VectorField Origin: " << Field.getOrig() << std::endl;
	std::cout << "SIM_VectorField Data0X: " << Field.getXField()->indexToPos({0, 0, 0}) << std::endl;
	std::cout << "SIM_VectorField Data0Y: " << Field.getYField()->indexToPos({0, 0, 0}) << std::endl;
	std::cout << "SIM_VectorField Data0Z: " << Field.getZField()->indexToPos({0, 0, 0}) << std::endl;
}
void HinaPE::print(const SIM_RawField &Field)
{
	std::cout << "SIM_RawField Origin: " << Field.getOrig() << std::endl;
	std::cout << "SIM_RawField Resolution: : " << Field.getVoxelRes() << std::endl;
	std::cout << "SIM_RawField VoxelSize: : " << Field.getVoxelSize() << std::endl;
	std::cout << "SIM_RawField Size: " << Field.getSize() << std::endl;
	std::cout << "SIM_RawField Data0 Pos: " << Field.indexToPos({0, 0, 0}) << std::endl;
}
void HinaPE::print(const CubbyFlow::CellCenteredScalarGrid3Ptr &Field)
{
	std::cout << "CubbyFlow::CellCenteredScalarGrid3 Data0 Pos: " << Field->DataPosition()({0, 0, 0}).x << ", " << Field->DataPosition()({0, 0, 0}).y << ", " << Field->DataPosition()({0, 0, 0}).z << std::endl;
}
void HinaPE::print(const CubbyFlow::VertexCenteredScalarGrid3Ptr &Field)
{
	std::cout << "CubbyFlow::VertexCenteredScalarGrid3 Data0 Pos: " << Field->DataPosition()({0, 0, 0}).x << ", " << Field->DataPosition()({0, 0, 0}).y << ", " << Field->DataPosition()({0, 0, 0}).z << std::endl;
}
void HinaPE::print(const CubbyFlow::CellCenteredVectorGrid3Ptr &Field)
{
	std::cout << "CubbyFlow::CellCenteredVectorGrid3 Data0 Pos: " << Field->DataPosition()({0, 0, 0}).x << ", " << Field->DataPosition()({0, 0, 0}).y << ", " << Field->DataPosition()({0, 0, 0}).z << std::endl;
}
void HinaPE::print(const CubbyFlow::VertexCenteredVectorGrid3Ptr &Field)
{
	std::cout << "CubbyFlow::VertexCenteredVectorGrid3 Data0 Pos: " << Field->DataPosition()({0, 0, 0}).x << ", " << Field->DataPosition()({0, 0, 0}).y << ", " << Field->DataPosition()({0, 0, 0}).z << std::endl;
}
void HinaPE::print(const CubbyFlow::FaceCenteredGrid3Ptr &Field)
{
	std::cout << "CubbyFlow::FaceCenteredGrid3 Data0 Pos: " << Field->DataPosition(0)({0, 0, 0}).x << ", " << Field->DataPosition(0)({0, 0, 0}).y << ", " << Field->DataPosition(0)({0, 0, 0}).z << std::endl;
	std::cout << "CubbyFlow::FaceCenteredGrid3 Data0 Pos: " << Field->DataPosition(1)({0, 0, 0}).x << ", " << Field->DataPosition(1)({0, 0, 0}).y << ", " << Field->DataPosition(1)({0, 0, 0}).z << std::endl;
	std::cout << "CubbyFlow::FaceCenteredGrid3 Data0 Pos: " << Field->DataPosition(2)({0, 0, 0}).x << ", " << Field->DataPosition(2)({0, 0, 0}).y << ", " << Field->DataPosition(2)({0, 0, 0}).z << std::endl;
}
void HinaPE::FieldUtil::_buildPartial(SIM_RawField *OutMarker, const SIM_RawField *BoundarySDF, const SIM_RawField *FluidSDF, const UT_JobInfo &info)
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
