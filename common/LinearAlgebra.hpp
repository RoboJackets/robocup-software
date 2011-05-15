#pragma once

/**
 * Provides some wrappers around matrices and linear algebra objects
 * and algorithms to avoid having too many separate math libraries
 */

#include <Eigen/Dense>

namespace LinAlg {

	/** basic types for general linear algebra */
	// dynamically sized components TODO: switch to fixed size
	typedef Eigen::VectorXf Vector;
	typedef Eigen::MatrixXf Matrix;

	/** constructors */
	inline Matrix eye(size_t n) {
		return Matrix::Identity(n, n);
	}

}
