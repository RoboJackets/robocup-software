#include "LinearAlgebra.hpp"

namespace LinAlg {

bool InvertMatrix(const Matrix& input, Matrix& inverse) {
	using namespace boost::numeric::ublas;
	typedef permutation_matrix<std::size_t> pmatrix;
	// create a working copy of the input
	Matrix A(input);
	// create a permutation matrix for the LU-factorization
	pmatrix pm(A.size1());

	// perform LU-factorization
	int res = lu_factorize(A,pm);
	if( res != 0 ) return false;

	// create identity matrix of "inverse"
	inverse.assign(identity_matrix<double>(A.size1()));

	// backsubstitute to get the inverse
	lu_substitute(A, pm, inverse);

	return true;
}

}
