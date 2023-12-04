#include "solver.h"

void HinaPE::PBFSolver::InitGeometry(SIM_Geometry *geometry)
{
	if (!geometry)
		return;

	UT_Vector3D gravity = Param.Vector3Params["gravity"];
	fpreal dt = Param.RealParams["dt"];
	fpreal particle_radius = Param.RealParams["particle_radius"];
	fpreal kernel_radius = Param.RealParams["kernel_radius"];
	fpreal target_density = Param.RealParams["target_density"];
	fpreal viscosity = Param.RealParams["viscosity"];
	fpreal vorticity = Param.RealParams["vorticity"];
}
void HinaPE::PBFSolver::SolveGeometry(SIM_Geometry *geometry, const SIM_Time &time)
{
	if (!geometry)
		return;

	UT_Vector3D gravity = Param.Vector3Params["gravity"];
	fpreal dt = Param.RealParams["dt"];
	fpreal particle_radius = Param.RealParams["particle_radius"];
	fpreal kernel_radius = Param.RealParams["kernel_radius"];
	fpreal target_density = Param.RealParams["target_density"];
	fpreal viscosity = Param.RealParams["viscosity"];
	fpreal vorticity = Param.RealParams["vorticity"];
}

