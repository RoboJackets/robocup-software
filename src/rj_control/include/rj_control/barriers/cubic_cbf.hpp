/**
 * @file cubic_cbf.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief A control barrier certificate using a cubic class-K function
 * @version 0.1
 * @date 2026-01-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>

#include <rj_constants/constants.hpp>

#include "rj_control/barriers/barrier_certificate.hpp"

namespace control {

class CubicCBF: public BarrierCertificate {
public:
    static constexpr int kConstraints = 2 * kNumShells;
    static constexpr double kSafetyRadius = 2 * kRobotRadius + 0.05;

    CubicCBF(int robot_id, double barrier_gain, double dt, bool goalie);

    rj_geometry::Twist apply(
        const WorldState& world_state,
        const PlayState& play_state,
        const FieldDimensions& field_dimensions,
        const rj_geometry::Twist& control_input,
        bool avoid_ball = false
    ) override;

    void calculate_robot_constraints(
        const WorldState& world_state,
        const PlayState& play_state,
        Eigen::Matrix<double, kConstraints, 2>& A,
        Eigen::Matrix<double, kConstraints, 1>& l,
        Eigen::Matrix<double, kConstraints, 1>& u
    ) const;

private:
    // The id of the robot being controlled
    int robot_id_;
    // The gain of the barrier
    double barrier_gain_;

    OsqpEigen::Solver solver_;
    Eigen::SparseMatrix<double> P_;
    Eigen::Matrix<double, 2, 1> q_;
    Eigen::Matrix<double, kConstraints, 2> A_;
    Eigen::Matrix<double, kConstraints, 1> l_;
    Eigen::Matrix<double, kConstraints, 1> u_;
};

} // namespace control