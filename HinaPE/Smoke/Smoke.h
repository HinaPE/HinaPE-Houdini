#ifndef HINAPE_SMOKE_H
#define HINAPE_SMOKE_H

/**
 * Grid Based Smoke Simulation
 *
 * NOT IMPLEMENTED YET
 */

#include <vector>
#include <UT/UT_Vector3.h>
#include <SIM/SIM_ScalarField.h>
#include <SIM/SIM_VectorField.h>
namespace HinaPE
{
//inline void for_each_voxel_index(SIM_ScalarField &sf, std::function<void(int, int, int)> func)
//{
//	int x_size = sf.getTotalVoxelRes();
//	int y_size = f.getVoxelRes().y();
//	int z_size = f.getVoxelRes().z();
//	for (int x = 0; x < x_size; x++)
//	{
//		for (int y = 0; y < y_size; y++)
//		{
//			for (int z = 0; z < z_size; z++)
//			{
//				func(x, y, z);
//			}
//		}
//	}
//}

struct SmokeNativeSolver
{
	static void emit(SIM_ScalarField *D, SIM_ScalarField *T, SIM_VectorField *V);
	static void apply_external_forces(float dt, SIM_VectorField *V);
	static void apply_viscosity(float dt);
	static void apply_pressure(float dt);
	static void apply_advection(float dt, SIM_ScalarField *SF, const SIM_VectorField *V);
};
}
#endif //HINAPE_SMOKE_H
