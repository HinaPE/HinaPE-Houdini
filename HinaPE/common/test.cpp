#include <Eigen/Sparse>
#include <iostream>

int main()
{
	// ����һ��ϡ�������Ϊϵ������A
	Eigen::SparseMatrix<double> A(4, 4);

	// ����һЩ����Ԫ��
	A.insert(0, 0) = 4;
	A.insert(0, 1) = 1;
	A.insert(1, 0) = 1;
	A.insert(1, 1) = 3;
	A.insert(2, 2) = 1;
	A.insert(2, 1) = -1;
	A.insert(1, 2) = -1;
	A.insert(3, 3) = 2;
	A.insert(3, 1) = 1;
	A.insert(1, 3) = 1;

	// �����Ҳ�����b
	Eigen::VectorXd b(4);
	b << 1, 2, 3, 4;

	// ��ʼ�������ݶ������
	Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper> cg;

	// ����
	cg.compute(A);

	// �������ɹ���ʹ����������Ax = b
	if (cg.info() == Eigen::Success)
	{
		Eigen::VectorXd x = cg.solve(b);
		// ������
		std::cout << "Solution:\n" << x << std::endl;
	} else
	{
		std::cout << "Decomposition failed" << std::endl;
	}

	return 0;
}
