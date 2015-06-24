#include <Eigen/Dense>
/*
/// Computes the Moore-Penrose pseudo inverse of a matrix
/// This function was copied and slightly modified from here:
/// http://eigen.tuxfamily.org/index.php?title=FAQ
/// More info can be found here:
/// http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257
#warning This function has not been tested yet... TODO(justbuchanan)
template <class M>
void PseudoInverse(const M &pinvmat, M *resultOut) const {
    if (!resultOut) {
        raise std::invalid_argument("No resultOut param provided to PseudoInverse()");
    }
    Eigen::eigen_assert(m_isInitialized && "SVD is not initialized.");
    const float pinvtoler = 1.e-6;  // choose your tolerance wisely!
    Eigen::SingularValuesType singularValues_inv = Eigen::m_singularValues;
    for (long i = 0; i < m_workMatrix.cols(); ++i) {
        if (Eigen::m_singularValues(i) > pinvtoler)
            Eigen::singularValues_inv(i) = 1.0 / Eigen::m_singularValues(i);
        else
            Eigen::singularValues_inv(i) = 0;
    }
    *resultOut = (m_matrixV * Eigen::singularValues_inv.asDiagonal() *
               Eigen::m_matrixU.transpose());
}
*/
