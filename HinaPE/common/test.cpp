#include <Eigen/Sparse>
#include <iostream>

int main()
{
	// 创建一个稀疏矩阵作为系数矩阵A
	Eigen::SparseMatrix<double> A(4, 4);

	// 插入一些非零元素
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

	// 创建右侧向量b
	Eigen::VectorXd b(4);
	b << 1, 2, 3, 4;

	// 初始化共轭梯度求解器
	Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper> cg;

	// 计算
	cg.compute(A);

	// 如果计算成功，使用求解器求解Ax = b
	if (cg.info() == Eigen::Success)
	{
		Eigen::VectorXd x = cg.solve(b);
		// 输出结果
		std::cout << "Solution:\n" << x << std::endl;
	} else
	{
		std::cout << "Decomposition failed" << std::endl;
	}

	return 0;
}
