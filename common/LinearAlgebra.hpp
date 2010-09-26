#pragma once

/**
 * Provides some wrappers around matrices and linear algebra objects
 * and algorithms to avoid having too many separate math libraries
 */

/** boost ublas for matrices/vectors */
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>

namespace LinAlg {

	/** basic types for general linear algebra */
	typedef boost::numeric::ublas::vector<double> Vector;
	typedef boost::numeric::ublas::matrix<double> Matrix;

	/** constructors */
	inline Matrix eye(size_t n) {
		return boost::numeric::ublas::identity_matrix<double>(n);
	}

	/** useful algorithms */

	/* Matrix inversion routine.
	Uses lu_factorize and lu_substitute in uBLAS to invert a matrix */
	bool InvertMatrix(const Matrix& input, Matrix& inverse);

	/** remapping operations to stay in our namespace */

	// matrix product
	inline Matrix prod(const Matrix& A, const Matrix& B) {
		return boost::numeric::ublas::prod(A,B);
	}

	// transpose of a matrix
	inline Matrix trans(const Matrix& A) {
		return boost::numeric::ublas::trans(A);
	}

}
