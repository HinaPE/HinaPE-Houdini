#include "GAS_Hina_Solver_Smoke.h"

#include <GAS/GAS_Advect.h>

GAS_HINA_SUBSOLVER_IMPLEMENT(
		Solver_Smoke,
		true,
		false,

		ACTIVATE_GAS_VELOCITY
				ACTIVATE_GAS_DENSITY
				ACTIVATE_GAS_TEMPERATURE
)

void print(SIM_ScalarField *f)
{
	std::cout << "Field Center: " << f->getCenter() << std::endl;
	std::cout << "Field Divisions: " << f->getDivisions() << std::endl;

	std::cout << "Field Size: " << f->getSize() << std::endl;
	std::cout << "Raw Size: " << f->getField()->getSize() << std::endl;

	std::cout << "Field Origin: " << f->getOrig() << std::endl;
	std::cout << "Raw Origin: " << f->getField()->getOrig() << std::endl;

	std::cout << "Raw Diameter: " << f->getField()->getVoxelDiameter() << std::endl;
	std::cout << "Raw Size: " << f->getField()->getVoxelSize() << std::endl;
	std::cout << "Raw Res: " << f->getField()->getVoxelRes() << std::endl;
	std::cout << "Raw Volume: " << f->getField()->getVoxelVolume() << std::endl;

	UT_BoundingBox box;
	f->getBBox(box);
	std::cout << "Field BBox: " << box << std::endl;
	std::cout << "Raw BBox Size: " << f->getField()->getBBoxSize() << std::endl;
	std::cout << "Raw BBox Origin: " << f->getField()->getBBoxOrig() << std::endl;
}

void print(SIM_VectorField *f)
{
	std::cout << "Field Center: " << f->getCenter() << std::endl;
	std::cout << "Field Divisions: " << f->getDivisions() << std::endl;

	std::cout << "Field Size: " << f->getSize() << std::endl;
	std::cout << "Raw Size x: " << f->getXField()->getSize() << std::endl;
	std::cout << "Raw Size y: " << f->getYField()->getSize() << std::endl;
	std::cout << "Raw Size z: " << f->getZField()->getSize() << std::endl;

	std::cout << "Field Origin: " << f->getOrig() << std::endl;
	std::cout << "Raw Origin x: " << f->getXField()->getOrig() << std::endl;
	std::cout << "Raw Origin y: " << f->getYField()->getOrig() << std::endl;
	std::cout << "Raw Origin z: " << f->getZField()->getOrig() << std::endl;

	std::cout << "Raw Diameter x: " << f->getXField()->getVoxelDiameter() << std::endl;
	std::cout << "Raw Diameter y: " << f->getYField()->getVoxelDiameter() << std::endl;
	std::cout << "Raw Diameter z: " << f->getZField()->getVoxelDiameter() << std::endl;
	std::cout << "Raw Size x: " << f->getXField()->getVoxelSize() << std::endl;
	std::cout << "Raw Size y: " << f->getYField()->getVoxelSize() << std::endl;
	std::cout << "Raw Size z: " << f->getZField()->getVoxelSize() << std::endl;
	std::cout << "Raw Res x: " << f->getXField()->getVoxelRes() << std::endl;
	std::cout << "Raw Res y: " << f->getYField()->getVoxelRes() << std::endl;
	std::cout << "Raw Res z: " << f->getZField()->getVoxelRes() << std::endl;
	std::cout << "Raw Volume x: " << f->getXField()->getVoxelVolume() << std::endl;
	std::cout << "Raw Volume y: " << f->getYField()->getVoxelVolume() << std::endl;
	std::cout << "Raw Volume z: " << f->getZField()->getVoxelVolume() << std::endl;

	UT_BoundingBox box;
	f->getBBox(box);
	std::cout << "Field BBox: " << box << std::endl;
	std::cout << "Raw BBox Size x: " << f->getXField()->getBBoxSize() << std::endl;
	std::cout << "Raw BBox Size y: " << f->getYField()->getBBoxSize() << std::endl;
	std::cout << "Raw BBox Size z: " << f->getZField()->getBBoxSize() << std::endl;
	std::cout << "Raw BBox Origin x: " << f->getXField()->getBBoxOrig() << std::endl;
	std::cout << "Raw BBox Origin y: " << f->getYField()->getBBoxOrig() << std::endl;
	std::cout << "Raw BBox Origin z: " << f->getZField()->getBBoxOrig() << std::endl;
}

void GAS_Hina_Solver_Smoke::_init()
{
	this->_inited = false;
}
void GAS_Hina_Solver_Smoke::_makeEqual(const GAS_Hina_Solver_Smoke *src)
{
	this->_inited = src->_inited;
}
bool GAS_Hina_Solver_Smoke::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_ScalarField *D = getScalarField(obj, GAS_NAME_DENSITY);
	SIM_ScalarField *T = getScalarField(obj, GAS_NAME_TEMPERATURE);
	SIM_VectorField *V = getVectorField(obj, GAS_NAME_VELOCITY);

	if (!_inited)
	{
		HinaPE::SmokeNativeSolver::emit(D, T, V);
		_inited = true;
	}
	else
	{
		HinaPE::SmokeNativeSolver::apply_external_forces(timestep, V);
		HinaPE::SmokeNativeSolver::apply_advection(timestep, D, V);
	}

	return true;
}
