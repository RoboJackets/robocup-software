#include "OfflineLqrController.hpp"
#include "logger.hpp"
#include <LinearAlgebra.hpp>

template<typename T>
T clamp(T value, T min, T max) {
    if (value > max) return max;
    else if (value < min) return min;
    else return value;
}

OfflineLqrController::OfflineLqrController(
    const RobotModel &robotModel, const RobotModelParams &robotModelParams,
    const LqrLookupTable<KType> *lookupTable)
    : _robotModel(robotModel),
      _robotModelParams(robotModelParams),
      _lookupTable(lookupTable) {
    // validate lookup table
    if (lookupTable == nullptr) {
        throw std::invalid_argument("LookupTable can't be null");
    }

    // Cache the value of pinv(B) to be used later
    _pinvB = PseudoInverse(_robotModel.B);
}

Eigen::Vector4f OfflineLqrController::computeControls(
    const Eigen::Vector3f &currVel, const Eigen::Vector3f &cmdVel) const {
    float dPhiDt = currVel[2];
    KType K;
    _lookupTable->lookup(dPhiDt, &K);
    RobotModel::AType A = _robotModel.A1 + _robotModel.A2 * dPhiDt;

    auto steadyStateTerm = -(_pinvB * A * cmdVel);
    auto correctionTerm = -K * (currVel - cmdVel);
    Eigen::Vector4f controlValues =  correctionTerm + steadyStateTerm;

    // limit output voltages to V_max
    for (int i = 0; i < 4; ++i) {
        controlValues[i] = clamp(controlValues[i], -_robotModelParams.V_max,
                                 _robotModelParams.V_max);
    }

    return controlValues;
}
