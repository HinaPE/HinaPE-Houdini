#include <iostream>
#include <vector>

struct Temp
{
	Temp(float _v) : v(_v) { std::cout << "Temp()" << std::endl; }
	virtual ~Temp()
	{
		std::cout << "~Temp()" << std::endl;
		v = 0.;
	}

	float v;
};

int main()
{
	Temp *a = nullptr;
	{
		std::vector<Temp> temp;
		temp.emplace_back(1.f);
		temp.emplace_back(2.f);
		temp.emplace_back(3.f);
		temp.emplace_back(4.f);

		a = &temp[3];
		std::cout << a->v << std::endl;
	}
	std::cout << a->v << std::endl;
	return 0;
}
