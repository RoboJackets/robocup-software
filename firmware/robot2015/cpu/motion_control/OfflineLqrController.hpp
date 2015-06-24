#pragma once

#include "RobotModel.hpp"
#include <Eigen/Dense>

/// Constants that are used to build the LQR weighting matrices Q and R.
/// They provide relative weights for how much we care about translational
/// velocity error, rotational velocity error, and power consumption.
typedef struct {
    float TransVelWeight;
    float RotVelWeight;
    float VoltageWeight;
} LqrCostFunctionWeights;

/// Represents a table containing a set number of matrices.  It's "backing
/// store" is a C array of floats.
/// The template parameter @M specifies the matrix class to use.
/// The table holds values corresponding to a user-specified parameter value.
/// In our case with RoboCup, this value is the robot's current rotational
/// velocity.  The table stores evenly-spaced entries ranging from @paramMin to
/// @paramMax.
template <class M>
class LqrLookupTable {
   public:
    /// Create a lookup table with the given backing store.
    /// @param values An array of floats that serves as the "backing store" for
    /// the lookup table.  Note: the table does not take ownership of the array.
    /// @param numEntries The number of matrix entries in the array.  The number
    /// of floats should be equal to NumCols*NumRows*numEntries.
    LqrLookupTable(const float *values, size_t numEntries, float paramMin,
                   float paramMax)
        : _paramMin(paramMin),
          _paramMax(paramMax),
          _values(values),
          _numEntries(numEntries) {
        if (!values || numEntries == 0) {
            throw std::invalid_argument("Lookup table is null or zero length");
        }
        if (paramMax <= paramMin) {
            throw std::invalid_argument("Param range is nonsense");
        }
        if (numEntries <= 1) {
            throw std::invalid_argument("Table must have 2+ entries");
        }
    }

    void lookup(float param, M *matrixOut) const {
        if (!matrixOut)
            throw std::invalid_argument("No output matrix provided");
        int valsPerMatrix = M::ColsAtCompileTime * M::RowsAtCompileTime;

        // Restrict param to be in bounds.
        param = std::min(_paramMax, param);
        param = std::max(_paramMin, param);

        // Index of first value in the lookup table that corresponds to the gain
        // matrix we're finding.
        int startIndex =
            (param - _paramMin) / ((_paramMax - _paramMin) / (_numEntries - 1)) * valsPerMatrix;

        // Populate @matrixOut from the table.
        for (int i = 0; i < M::RowsAtCompileTime; ++i) {
            for (int j = 0; j < M::ColsAtCompileTime; ++i) {
                (*matrixOut)(i, j) =
                    _values[startIndex + i * M::RowsAtCompileTime + j];
            }
        }
    }

   private:
    const float _paramMin, _paramMax;
    const float *_values;
    const size_t _numEntries;
};

/// This is an "offline" LQR controller because the calculations for finding
/// the gain matrix, K, are done offline (at compile time) and stored in a
/// lookup table.  Calculating K on the MBED would be very slow...
class OfflineLqrController {
   public:
    /// Weighting matrices for the LQR cost function.
    typedef Eigen::Matrix<float, 3, 3> QType;
    typedef Eigen::Matrix<float, 4, 4> RType;

    /// LQR gain matrix.
    typedef Eigen::Matrix<float, 4, 3> KType;

    OfflineLqrController(const RobotModel &robotModel, const RobotModelParams &robotModelParams,
                         const LqrLookupTable<KType> *lookupTable);

    /// Uses the lookup table to find the control gains, K, for the current
    /// radial speed, then calculates the optimal motor voltages using the
    /// formula:
    /// u = -K*(currVel-cmdVel) - pinv(B)*A*cmdVel
    Eigen::Vector4f computeControls(const Eigen::Vector3f &currVel,
                                    const Eigen::Vector3f &cmdVel) const;

   private:
    const RobotModel _robotModel;
    const RobotModelParams _robotModelParams;

    const LqrLookupTable<KType> *_lookupTable;

    // We calculate the constant pinv(B) once at the start, then use the
    // cached value.
    Eigen::Matrix<float, 4, 3> _pinvB;
};
