#include <gtest/gtest.h>
#include <modeling/rbpf/Rbpf.hpp>

using namespace rbpf;

/* ************************************************************************* */
TEST( testRbpf, full_rbpf ) {
	VectorNd X; X.setOnes();
	MatrixNNd P; P = 0.1 * MatrixNNd::Identity();
	size_t k = 2;
	Rbpf filter(X, P, k);
}

