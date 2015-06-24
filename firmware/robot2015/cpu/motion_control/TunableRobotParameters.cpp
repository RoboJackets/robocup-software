#include "TunableRobotParameters.hpp"

#include <Geometry2d/util.h>

// clang-format off
const RobotModelParams Robot2015ModelParams = {
    .M_bot = 2.205,
    .I_bot = 0.00745879949,
    .g = 2.0 / 7.0,
    .r = 0.0285623,
    .L = 0.0798576,
    .Rt = 0.464,
    .K_e = 30.0 / (380 * M_PI),
    .K_t = 0.0251,
    .K_f = 0.0001,
    .I_asm = 2.43695253e-5,
    .V_max = 18,
    .WheelAngles = {38 * DegreesToRadians,
                       142 * DegreesToRadians,
                       225 * DegreesToRadians,
                       315 * DegreesToRadians}
};
// clang-format on

const RobotModel Robot2015SystemModel(Robot2015ModelParams);

// clang-format off
const LqrCostFunctionWeights Robot2015LqrCostFunctionWeights = {
    .TransVelWeight = 5,
    .RotVelWeight = 1,
    .VoltageWeight = 0.5
};
// clang-format on

const OfflineLqrController::QType Robot2015LqrQ = []() {
    OfflineLqrController::QType q;
    // clang-format off
    q << Robot2015LqrCostFunctionWeights.TransVelWeight, 0, 0,
         0, Robot2015LqrCostFunctionWeights.TransVelWeight, 0,
         0, 0, Robot2015LqrCostFunctionWeights.RotVelWeight;
    // clang-format on
    return q;
}();

const OfflineLqrController::RType Robot2015LqrR = []() {
    OfflineLqrController::RType r =
        OfflineLqrController::RType::Identity() *
        Robot2015LqrCostFunctionWeights.VoltageWeight;
    return r;
}();

const float Robot2015LqrLookupTableMinRotVel =
    -Robot2015LqrLookupTableMaxRotVel;
const float Robot2015LqrLookupTableMaxRotVel = M_PI*2;

const size_t Robot2015LqrLookupTableNumEntries = 100;
