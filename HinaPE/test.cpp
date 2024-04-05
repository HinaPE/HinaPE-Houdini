#include <UT/UT_Vector3.h>
#include <SIM/SIM_RawField.h>

int main()
{
	SIM_RawField F;
	F.init(SIM_SAMPLE_CENTER, {0, 0, 0}, {3, 3, 3}, 3, 3, 3);
	F.makeConstant(0);
	F.setCellValue(1, 0, 1, 0.5);
	F.setCellValue(1, 1, 1, 1);
	F.setCellValue(1, 2, 1, 1.5);
	std::cout << "Center Value: " << F.getValue(F.indexToPos({1, 1, 1})) << std::endl;
	std::cout << "HDK Gradient: " << F.getGradientAtIndex(1, 1, 1) << std::endl;
	std::cout << "HDK Laplacian: " << F.getLaplacianAtIndex(1, 1, 1) << std::endl;
	return 0;
}
