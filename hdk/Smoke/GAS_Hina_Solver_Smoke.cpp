#include "GAS_Hina_Solver_Smoke.h"

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
	std::cout << "Raw Size x: " << f->getField(0)->getSize() << std::endl;
	std::cout << "Raw Size y: " << f->getField(1)->getSize() << std::endl;
	std::cout << "Raw Size z: " << f->getField(2)->getSize() << std::endl;

	std::cout << "Field Origin: " << f->getOrig() << std::endl;
	std::cout << "Raw Origin x: " << f->getField(0)->getOrig() << std::endl;
	std::cout << "Raw Origin y: " << f->getField(1)->getOrig() << std::endl;
	std::cout << "Raw Origin z: " << f->getField(2)->getOrig() << std::endl;

	std::cout << "Raw Diameter x: " << f->getField(0)->getVoxelDiameter() << std::endl;
	std::cout << "Raw Diameter y: " << f->getField(1)->getVoxelDiameter() << std::endl;
	std::cout << "Raw Diameter z: " << f->getField(2)->getVoxelDiameter() << std::endl;
	std::cout << "Raw Size x: " << f->getField(0)->getVoxelSize() << std::endl;
	std::cout << "Raw Size y: " << f->getField(1)->getVoxelSize() << std::endl;
	std::cout << "Raw Size z: " << f->getField(2)->getVoxelSize() << std::endl;
	std::cout << "Raw Res x: " << f->getField(0)->getVoxelRes() << std::endl;
	std::cout << "Raw Res y: " << f->getField(1)->getVoxelRes() << std::endl;
	std::cout << "Raw Res z: " << f->getField(2)->getVoxelRes() << std::endl;
	std::cout << "Raw Volume x: " << f->getField(0)->getVoxelVolume() << std::endl;
	std::cout << "Raw Volume y: " << f->getField(1)->getVoxelVolume() << std::endl;
	std::cout << "Raw Volume z: " << f->getField(2)->getVoxelVolume() << std::endl;

	UT_BoundingBox box;
	f->getBBox(box);
	std::cout << "Field BBox: " << box << std::endl;
	std::cout << "Raw BBox Size x: " << f->getField(0)->getBBoxSize() << std::endl;
	std::cout << "Raw BBox Size y: " << f->getField(1)->getBBoxSize() << std::endl;
	std::cout << "Raw BBox Size z: " << f->getField(2)->getBBoxSize() << std::endl;
	std::cout << "Raw BBox Origin x: " << f->getField(0)->getBBoxOrig() << std::endl;
	std::cout << "Raw BBox Origin y: " << f->getField(1)->getBBoxOrig() << std::endl;
	std::cout << "Raw BBox Origin z: " << f->getField(2)->getBBoxOrig() << std::endl;
}

void GAS_Hina_Solver_Smoke::_init() {}
void GAS_Hina_Solver_Smoke::_makeEqual(const GAS_Hina_Solver_Smoke *src) {}
bool GAS_Hina_Solver_Smoke::_solve(SIM_Engine &engine, SIM_Object *obj, SIM_Time time, SIM_Time timestep)
{
	SIM_ScalarField *density = getScalarField(obj, GAS_NAME_DENSITY);
	SIM_ScalarField *temperature = getScalarField(obj, GAS_NAME_TEMPERATURE);
	SIM_VectorField *velocity = getVectorField(obj, GAS_NAME_VELOCITY);

	print(velocity);

	return true;
}
