#include <Eigen/Dense>

/// Computes the Moore-Penrose pseudo inverse of a matrix
/// This function was copied and slightly modified from here:
/// http://eigen.tuxfamily.org/index.php?title=FAQ
/// More info can be found here:
/// http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257
#warning This function has not been tested yet... TODO(justbuchanan)
#warning this function has some sort of size mismatch:
// assertion "diagonal.diagonal().size() == (ProductOrder == OnTheLeft ? matrix.rows() : matrix.cols())" failed: file "/usr/include/eigen3/Eigen/src/Core/DiagonalProduct.h", line 55, function: Eigen::DiagonalProduct<MatrixType, DiagonalType, ProductOrder>::DiagonalProduct(const MatrixType&, const DiagonalType&) [with MatrixType = Eigen::Matrix<float, -1, -1>; DiagonalType = Eigen::DiagonalWrapper<const Eigen::Matrix<float, -1, 1> >; int ProductOrder = 2]

template <typename M>
Eigen::Matrix<typename M::Scalar, M::ColsAtCompileTime, M::RowsAtCompileTime> PseudoInverse(
    const M& m, typename M::Scalar epsilon = 1E-9) {
    typedef Eigen::JacobiSVD<Eigen::MatrixXf> SVD;
    SVD svd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
    typedef typename SVD::SingularValuesType SingularValuesType;
    const SingularValuesType singVals = svd.singularValues();
    SingularValuesType invSingVals = singVals;
    for (int i = 0; i < singVals.rows(); i++) {
        if (singVals(i) <= epsilon) {
            invSingVals(i) = 0.0;  // FIXED can not be safely inverted
        } else {
            invSingVals(i) = 1.0 / invSingVals(i);
        }
    }
    return svd.matrixV() * invSingVals.asDiagonal() * svd.matrixU().transpose();
}
