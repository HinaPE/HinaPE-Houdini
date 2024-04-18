#include <UT/UT_DSOVersion.h> // Very Important!!! Include this!!!

#include <GAS_CFL_SubStep.h>
#include <Base/SIM_Hina_Particles.h>
#include <Boundary/SIM_Hina_Particles_Akinci.h>
#include <Boundary/SIM_Hina_SDF_Boundary.h>
#include <DFSPH/SIM_Hina_Particles_DFSPH.h>
#include <DFSPH/GAS_Hina_Solver_DFSPH.h>
#include <PBF/SIM_Hina_Particles_PBF.h>
#include <PBF/GAS_Hina_Solver_PBF.h>
#include <Rigid/SIM_Hina_RigidBody.h>
#include <Rigid/GAS_Hina_Solver_Rigid.h>
#include <Smoke/GAS_Hina_GridAdvect.h>
#include <Smoke/GAS_Hina_GridBoundarySolver.h>
#include <Smoke/GAS_Hina_GridBuildMarker.h>
#include <Smoke/GAS_Hina_GridDiffusion.h>
#include <Smoke/GAS_Hina_GridExternalForce.h>
#include <Smoke/GAS_Hina_GridPressure.h>
#include <Smoke/GAS_Hina_GridSourceEmitter.h>
#include <Smoke/GAS_Hina_ShowInfo.h>

#include <_Test/GAS_Hina_Test.h>

void initializeSIM(void *)
{
	// Particle Based Classes
	IMPLEMENT_DATAFACTORY(GAS_CFL_SubStep)
	IMPLEMENT_DATAFACTORY(SIM_Hina_Particles)
	IMPLEMENT_DATAFACTORY(SIM_Hina_Particles_DFSPH)
	IMPLEMENT_DATAFACTORY(GAS_Hina_Solver_DFSPH)
	IMPLEMENT_DATAFACTORY(SIM_Hina_Particles_PBF)
	IMPLEMENT_DATAFACTORY(GAS_Hina_Solver_PBF)
	IMPLEMENT_DATAFACTORY(SIM_Hina_Particles_Akinci)
	IMPLEMENT_DATAFACTORY(SIM_Hina_SDF_Boundary)
	IMPLEMENT_DATAFACTORY(SIM_Hina_RigidBody)

	// Grid Based Classes
	IMPLEMENT_DATAFACTORY(GAS_Hina_Solver_Rigid)
	IMPLEMENT_DATAFACTORY(GAS_Hina_GridAdvect)
	IMPLEMENT_DATAFACTORY(GAS_Hina_GridBoundarySolver)
	IMPLEMENT_DATAFACTORY(GAS_Hina_GridBuildMarker)
	IMPLEMENT_DATAFACTORY(GAS_Hina_GridDiffusion)
	IMPLEMENT_DATAFACTORY(GAS_Hina_GridPressure)
	IMPLEMENT_DATAFACTORY(GAS_Hina_GridExternalForce)
	IMPLEMENT_DATAFACTORY(GAS_Hina_GridSourceEmitter)
	IMPLEMENT_DATAFACTORY(GAS_Hina_ShowInfo)

	// TEST CLASSES
	IMPLEMENT_DATAFACTORY(GAS_Hina_Test)
}
