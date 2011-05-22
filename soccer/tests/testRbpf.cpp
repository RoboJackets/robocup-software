#include <gtest/gtest.h>
#include <modeling/rbpf/Rbpf.hpp>

using namespace rbpf;

/* ************************************************************************* */
TEST( testRbpf, full_rbpf ) {
	VectorNf X; X.setOnes();
	MatrixNNf P; P = 0.1 * MatrixNNf::Identity();
	size_t k = 2;
	Rbpf filter(X, P, k);
}

