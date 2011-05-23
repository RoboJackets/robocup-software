#include <gtest/gtest.h>
#include "Eigen/Dense"
#include <modeling/rbpf/rbpfMatrices.h>

/* ************************************************************************* */
TEST( testEigen, eigen_compilation ) {
	Eigen::MatrixXd matrix1 = Eigen::MatrixXd::Constant(2, 3, 1.23);
	Eigen::MatrixXd matrix2(2, 3);
	matrix2 << 1.23, 1.23, 1.23, 1.23, 1.23, 1.23;
	EXPECT_EQ(matrix1, matrix2);
}

/* ************************************************************************* */
TEST( testEigen, identity ) {
	using namespace rbpf;
	MatrixSNd H;
	H.setIdentity();
	MatrixSNd expH;
	expH << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
	EXPECT_EQ(expH, H);
}

