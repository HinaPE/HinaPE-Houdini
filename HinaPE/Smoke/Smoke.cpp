#include "Smoke.h"
#include <UT/UT_MatrixSolver.h>
#include <memory>

void HinaPE::SmokeNativeSolver::test(SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V)
{
	UT_VoxelArrayF &u = *(*V).getField(0)->fieldNC();
	UT_VoxelArrayF &v = *(*V).getField(1)->fieldNC();
	UT_VoxelArrayF &w = *(*V).getField(2)->fieldNC();
	UT_VoxelArrayIteratorF vit;
	{
		vit.setArray(&u);
		for (vit.rewind(); !vit.atEnd(); vit.advance())
		{
			std::cout << vit.x() << " " << vit.y() << " " << vit.z() << " " << vit.getValue() << std::endl;
		}
	}
	std::cout << "----------------" << std::endl;
	{
		vit.setArray(&v);
		for (vit.rewind(); !vit.atEnd(); vit.advance())
		{
			std::cout << vit.x() << " " << vit.y() << " " << vit.z() << " " << vit.getValue() << std::endl;
		}
	}
	std::cout << "----------------" << std::endl;
	{
		vit.setArray(&w);
		for (vit.rewind(); !vit.atEnd(); vit.advance())
		{
			std::cout << vit.x() << " " << vit.y() << " " << vit.z() << " " << vit.getValue() << std::endl;
		}
	}
	std::cout << "----------------" << std::endl;
}
void HinaPE::SmokeNativeSolver::emit(SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V)
{
	UT_Vector3 voxel_dim = D->getVoxelSize();
	UT_Vector3 origin = D->getOrig();
	UT_Vector3 divisions = D->getDivisions();
	UT_Vector3 center = D->getCenter();
	UT_Vector3 size = D->getSize();

	std::cout << "Voxel Size: " << voxel_dim << std::endl;
	std::cout << "Origin: " << origin << std::endl;
	std::cout << "Divisions: " << divisions << std::endl;
	std::cout << "Center: " << center << std::endl;
	std::cout << "Size: " << size << std::endl;

	fpreal32 r = 0.3f * (divisions * voxel_dim).x();
	UT_Vector3 source = origin + 0.5 * size;
	source.y() = origin.y() + r;

	UT_VoxelArrayF *_D = D->getField()->fieldNC();
	for (int x = 0; x < divisions.x(); x++)
	{
		for (int y = 0; y < divisions.y(); y++)
		{
			for (int z = 0; z < divisions.z(); z++)
			{
				UT_Vector3F pos_grid, pos_world;
				_D->indexToPos(x, y, z, pos_grid);
				pos_world = pos_grid * size + origin;
				if ((pos_world - source).length() < r)
				{
					_D->setValue(x, y, z, 1.0f);
				}
			}
		}
	}

	UT_VoxelArrayF *velY = V->getYField()->fieldNC();
	for (int x = 0; x < divisions.x(); x++)
	{
		for (int y = 0; y < divisions.y(); y++)
		{
			for (int z = 0; z < divisions.z(); z++)
			{
				fpreal32 old_value = velY->getValue(x, y, z);
				fpreal32 new_value = old_value + -1;
				velY->setValue(x, y, z, new_value);
			}
		}
	}
}
void HinaPE::SmokeNativeSolver::apply_advection(float dt, SIM_ScalarField *SF, SIM_ScalarField *Co, const SIM_VectorField *V)
{
	SF->advect(V, dt, Co->getField(), SIM_ADVECT_MIDPOINT, 0.4f);
}
//void HinaPE::SmokeNativeSolver::apply_external_forcePartial(const UT_JobInfo &info)
//{
//	int i, n;
//	for (info.divideWork(10, i, n); i < n; i++)
//	{
//		std::cout << "Thread: " << std::endl;
//	}
//}


void HinaPE::SmokeNativeSolver::apply_external_forcePartial(float dt, const UT_JobInfo &info)
{
	SIM_RawField viscfielddata;
	viscfielddata = *V->getXField();
}
void HinaPE::SmokeNativeSolver::apply_viscosityPartial(float dt, const UT_JobInfo &info)
{

}
void HinaPE::SmokeNativeSolver::apply_pressurePartial(float dt, const UT_JobInfo &info)
{

}
