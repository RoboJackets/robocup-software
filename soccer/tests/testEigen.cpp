#include <gtest/gtest.h>
#include "Eigen/Dense"

/* ************************************************************************* */
TEST( testEigen, eigen_compilation ) {
	Eigen::MatrixXd matrix1 = Eigen::MatrixXd::Constant(2, 3, 1.23);
	Eigen::MatrixXd matrix2(2, 3);
	matrix2 << 1.23, 1.23, 1.23, 1.23, 1.23, 1.23;
	EXPECT_EQ(matrix1, matrix2);
}

