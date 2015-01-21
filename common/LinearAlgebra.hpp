#pragma once

#include <Eigen/Dense>


namespace LinAlg {

	/** constructors */
	inline Eigen::MatrixXf eye(size_t n) {
		return MatrixXf::Identity(n, n);
	}

}
