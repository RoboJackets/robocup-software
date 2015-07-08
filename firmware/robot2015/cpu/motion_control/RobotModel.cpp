#include "RobotModel.hpp"

RobotModel::RobotModel(const RobotModelParams &params) {
    // TODO(justbuchanan): add link to derivations

    // Inertia matrix
    Eigen::Matrix<float, 3, 3> J;
    // clang-format off
    J << params.M_bot, 0, 0,
         0, params.M_bot, 0,
         0, 0, params.I_bot;
    // clang-format on

    // Geometery matrix
    Eigen::Matrix<float, 3, 4> G;
    G << -sinf(params.WheelAngles[0]), -sinf(params.WheelAngles[1]),
        -sinf(params.WheelAngles[2]), -sinf(params.WheelAngles[3]),
        cosf(params.WheelAngles[0]), cosf(params.WheelAngles[1]),
        cosf(params.WheelAngles[2]), cosf(params.WheelAngles[3]),
        1.0 / params.L, 1.0 / params.L, 1.0 / params.L, 1.0 / params.L;

    A1 = -(params.K_e * params.K_t / params.Rt + params.K_f) /
         (params.M_bot * powf(params.g, 2) * powf(params.r, 2) + params.I_asm) *
         AType::Identity();

    // clang-format off
    A2 << 0, 1, 0,
         -1, 0, 0,
          0, 0, 0;
    // clang-format on
    A2 *= (params.M_bot * powf(params.g, 2) * powf(params.r, 2)) /
          (params.M_bot * powf(params.g, 2) * powf(params.r, 2) + params.I_asm);

    B = params.g * params.r * params.K_t * G / params.Rt /
        (params.M_bot * powf(params.g, 2) * powf(params.r, 2) + params.I_asm);
}

RobotModel::RobotModel(const AType &a1, const AType &a2,
                                   const BType &b)
    : A1(a1), A2(a2), B(b) {}
