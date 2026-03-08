/**
 * @file linear_cbf.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief A control barrier function that uses a linear class-K function
 * @version 0.1
 * @date 2025-12-23
 * 
 * @copyright Copyright (c) 2025
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

/**
 * @brief A linear decentralized safety barrier certificate
 * 
 */
class LinearCBF : public BarrierCertificate {
public:
    static constexpr int kConstraints = 2 * kNumShells // Robot Constraints
        + 3 // our goal constraints
        + 3 // their goal constraints
        + 1 // ball constraint
        + 2 // ball placement constraints
        + 4; // velocity constraints
    // The index our robot constraints start
    static constexpr size_t kOurRobotsConstraintIdx = 0;
    // The index their robot constraints start
    static constexpr size_t kTheirRobotsConstraintIdx = kNumShells;
    // The index our goal area constraints start
    static constexpr size_t kOurGoalConstraintsIdx = kTheirRobotsConstraintIdx + kNumShells;
    // The index their goal area constraints start
    static constexpr size_t kTheirGoalConstraintsIdx = kOurRobotsConstraintIdx + 3;
    // The index the ball constraint starts
    static constexpr size_t kBallConstraintIdx = kTheirGoalConstraintsIdx + 3;
    // The index the ball placement constraints start
    static constexpr size_t kBallPlacementConstraintIdx = kBallConstraintIdx + 1;
    // The index the velocity constraints start
    static constexpr size_t kVelocityConstraintIdx = kBallPlacementConstraintIdx + 2;

    // The safe distance between robots
    static constexpr double kSafeDistanceRobots = kRobotRadius * 2 + 0.01;
    // The safe distance from the goal outside of stop
    static constexpr double kSafeDistanceGoal = kRobotRadius + 0.01;
    // The safe distance from the goal during stop
    static constexpr double kSafeDistanceGoalStop = kSafeDistanceGoal + 0.2;
    // The safe distance from the ball during kick off, free kick, and ball placement
    static constexpr double kSafeDistanceBall = kRobotRadius + 0.51;
    // The safe distance when moving if we are avoiding the ball
    static constexpr double kAvoidBallDistance = kRobotRadius + 0.1;

    LinearCBF(int robot_id, double control_gain, double dt, bool goalie = false); //NOLINT(readability-identifier-length)

    rj_geometry::Twist apply(
        const WorldState& world_state,
        const PlayState& play_state,
        const FieldDimensions& field_dimensions,
        const rj_geometry::Twist& control_input,
        bool avoid_ball = false
    ) override;

    /**
     * @brief Calculate robot constraints for the barrier certificate
     * 
     * @param world_state
     * @param play_state
     * @param A 
     * @param l 
     * @param r 
     */
    void calculate_robot_constraints(
        const WorldState& world_state,
        const PlayState& play_state,
        Eigen::Matrix<double, kConstraints, 2>& A, //NOLINT(readability-identifier-naming, readability-identifier-length)
        Eigen::Matrix<double, kConstraints, 1>& l, //NOLINT(readability-identifier-naming, readability-identifier-length)
        Eigen::Matrix<double, kConstraints, 1>& u //NOLINT(readability-identifier-naming, readability-identifier-length)
    ) const;

    /**
     * @brief Calculate goal constraints for our goal
     * 
     * @param world_state 
     * @param play_state 
     * @param field_dimensions 
     * @param A 
     * @param l 
     * @param u 
     */
    void calculate_our_goal_constraints(
        const WorldState& world_state,
        const PlayState& play_state,
        const FieldDimensions& field_dimensions,
        Eigen::Matrix<double, kConstraints, 2>& A, //NOLINT(readability-identifier-naming, readability-identifier-length)
        Eigen::Matrix<double, kConstraints, 1>& l, //NOLINT(readability-identifier-naming, readability-identifier-length)
        Eigen::Matrix<double, kConstraints, 1>& u //NOLINT(readability-identifier-naming, readability-identifier-length)
    ) const;

    /**
     * @brief Calculate goal constraints for their goal
     * 
     * @param world_state 
     * @param play_state 
     * @param field_dimensions 
     * @param A 
     * @param l 
     * @param u 
     */
    void calculate_their_goal_constraints(
        const WorldState& world_state,
        const PlayState& play_state,
        const FieldDimensions& field_dimensions,
        Eigen::Matrix<double, kConstraints, 2>& A, //NOLINT(readability-identifier-naming, readability-identifier-length)
        Eigen::Matrix<double, kConstraints, 1>& l, //NOLINT(readability-identifier-naming, readability-identifier-length)
        Eigen::Matrix<double, kConstraints, 1>& u //NOLINT(readability-identifier-naming, readability-identifier-length)
    ) const;

    /**
     * @brief Calculate ball position constraints for the barrier certificate
     * 
     * @param world_state 
     * @param play_state 
     * @param A 
     * @param l 
     * @param u 
     */
    void calculate_ball_position_constraints(
        const WorldState& world_state,
        const PlayState& play_state,
        Eigen::Matrix<double, kConstraints, 2>& A, //NOLINT(readability-identifier-naming, readability-identifier-length)
        Eigen::Matrix<double, kConstraints, 1>& l, //NOLINT(readability-identifier-naming, readability-identifier-length)
        Eigen::Matrix<double, kConstraints, 1>& u, //NOLINT(readability-identifier-naming, readability-identifier-length)
        bool avoid_ball = false
    ) const;

    /**
     * @brief Calculate ball placement constraints for the barrier certificate
     * 
     * @param world_state 
     * @param play_state 
     * @param A 
     * @param l 
     * @param u 
     */
    void calculate_ball_placement_constraints(
        const WorldState& world_state,
        const PlayState& play_state,
        Eigen::Matrix<double, kConstraints, 2>& A, //NOLINT(readability-identifier-naming, readability-identifier-length)
        Eigen::Matrix<double, kConstraints, 1>& l, //NOLINT(readability-identifier-naming, readability-identifier-length)
        Eigen::Matrix<double, kConstraints, 1>& u //NOLINT(readability-identifier-naming, readability-identifier-length)
    ) const;

    /**
     * @brief Calculate the velocity constraints as a result of the current
     * play state
     * 
     * @param play_state 
     * @param A 
     * @param l 
     * @param u 
     */
    static void calculate_velocity_constraints(
        const PlayState& play_state,
        Eigen::Matrix<double, kConstraints, 2>& A, //NOLINT(readability-identifier-naming, readability-identifier-length)
        Eigen::Matrix<double, kConstraints, 1>& l, //NOLINT(readability-identifier-naming, readability-identifier-length)
        Eigen::Matrix<double, kConstraints, 1>& u //NOLINT(readability-identifier-naming, readability-identifier-length)
    );

private:
    // The OSQP Solver
    OsqpEigen::Solver solver_;
    // Quadratic Cost Matrix
    Eigen::SparseMatrix<double> P_; // NOLINT(readability-identifier-naming)
    // Linear Cost Vector
    Eigen::Matrix<double, 2, 1> q_;
    // Constraints Matrix
    Eigen::Matrix<double, kConstraints, 2> A_; //NOLINT(readability-identifier-naming)
    // Constraints lower bounds
    Eigen::Matrix<double, kConstraints, 1> l_;
    // Constraints upper bounds
    Eigen::Matrix<double, kConstraints, 1> u_;
};

} // namespace control