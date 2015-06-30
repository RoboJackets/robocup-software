#include <Eigen/Dense>
// #include <Eigen/SVD>

/// Computes the Moore-Penrose pseudo inverse of a matrix
/// This function was copied and slightly modified from here:
/// http://eigen.tuxfamily.org/index.php?title=FAQ
/// More info can be found here:
/// http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257
#warning This function has not been tested yet... TODO(justbuchanan)
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
