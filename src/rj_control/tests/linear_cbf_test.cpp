#include <gtest/gtest.h>

#include <rj_geometry/pose.hpp>
#include <rj_common/world_state.hpp>
#include <rj_common/game_state.hpp>
#include <rj_common/field_dimensions.hpp>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <rj_control/barriers/linear_cbf.hpp>

namespace control {

WorldState initialize_world_state(
    std::vector<rj_geometry::Pose> our_robot_poses,
    std::vector<rj_geometry::Twist> our_robot_velocities,
    std::vector<rj_geometry::Pose> their_robot_poses,
    std::vector<rj_geometry::Twist> their_robot_velocities,
    rj_geometry::Point ball_position,
    rj_geometry::Point ball_velocity
) {
    RJ::Time now = RJ::now();

    // Create our robot states
    std::vector<RobotState> our_robot_states;
    for (size_t i = 0; i < our_robot_poses.size(); i++) {
        our_robot_states.emplace_back(RobotState(
            our_robot_poses[i],
            our_robot_velocities[i],
            now,
            true
        ));
    }
    for (size_t i = our_robot_poses.size(); i < kNumShells; i++) {
        our_robot_states.emplace_back(RobotState());
    }

    // Create their robot states
    std::vector<RobotState> their_robot_states;
    for (size_t i = 0; i < their_robot_poses.size(); i++) {
        their_robot_states.emplace_back(RobotState(
            their_robot_poses[i],
            their_robot_velocities[i],
            now,
            true
        ));
    }
    for (size_t i = their_robot_poses.size(); i < kNumShells; i++) {
        their_robot_states.emplace_back(RobotState());
    }

    // Create the ball state
    BallState ball_state = BallState(
        ball_position,
        ball_velocity,
        now
    );

    return {their_robot_states, our_robot_states, ball_state};
}

//NOLINTNEXTLINE
TEST(LinearCBF, CalculateRobotConstraints) {
    FieldDimensions field_dimensions = FieldDimensions::kDefaultDimensions;
    PlayState play_state = PlayState::playing();
    WorldState world_state = initialize_world_state(
        {{
            {{1.0, 1.0, 0.0}},
            {{1.3, 1.3, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{-1.0, -1.0, 0.0}}
        }},
        {{
            {{0.7, 0.7, 0.0}},
            {{0.1, 0.1, 0.0}}
        }},
        {{
            {{1.0, 1.0, 0.0}},
            {{0.0, 0.0, 0.0}}
        }},
        {0, 0},
        {0, 0}
    );

    Eigen::Matrix<double, LinearCBF::kConstraints, 2> A; //NOLINT(readability-identifier-length, readability-identifier-naming)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> l; //NOLINT(readability-identifier-length)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> u; //NOLINT(readability-identifier-length)

    LinearCBF cbf = LinearCBF(0, 1.0, 1.0);
    cbf.calculate_robot_constraints(world_state, play_state, A, l, u);

    // Check the constraints for ourself (they should all be zero)
    ASSERT_FLOAT_EQ(A(0, 0), 0);
    ASSERT_FLOAT_EQ(A(0, 1), 0);
    ASSERT_EQ(l(0), -1e9);
    ASSERT_EQ(u(0), 1e9);

    // Check the constraints for our robot 1
    ASSERT_FLOAT_EQ(A(1, 0), 0.3);
    ASSERT_FLOAT_EQ(A(1, 1), 0.3);
    double h_12 = sqrt(0.18) * (sqrt(0.18) - LinearCBF::kSafeDistanceRobots);
    ASSERT_FLOAT_EQ(u(1), -0.6 + h_12);
    ASSERT_EQ(l(1), -1e9);

    // Check the constraints for their robot 0
    ASSERT_FLOAT_EQ(A(kNumShells, 0), -0.3);
    ASSERT_FLOAT_EQ(A(kNumShells, 1), -0.3);
    double h_13 = sqrt(0.18) * (sqrt(0.18) - LinearCBF::kSafeDistanceRobots);
    ASSERT_FLOAT_EQ(u(kNumShells), -0.6 + h_13);
    ASSERT_EQ(l(kNumShells), -1e9);

    // Check the constraints for their robot 1
    ASSERT_FLOAT_EQ(A(kNumShells+1, 0), -0.9);
    ASSERT_FLOAT_EQ(A(kNumShells+1, 1), -0.9);
    double h_14 = sqrt(1.62) * (sqrt(1.62) - LinearCBF::kSafeDistanceRobots);
    ASSERT_FLOAT_EQ(u(kNumShells + 1), 0 + h_14);
    ASSERT_EQ(l(kNumShells), -1e9);

    // Check the constraints for our invisible robots
    for (size_t i = 2; i < kNumShells; i++) {
        ASSERT_FLOAT_EQ(A(i, 0), -1.0);
        ASSERT_FLOAT_EQ(A(i, 1), -1.0);
        ASSERT_FLOAT_EQ(u(i), 1e9);
        ASSERT_FLOAT_EQ(l(i), -1e9);
    }

    // Check the constraints for their invisible robots
    for (size_t i = 2; i < kNumShells; i++) {
        ASSERT_FLOAT_EQ(A(kNumShells + i, 0), -1.0);
        ASSERT_FLOAT_EQ(A(kNumShells + i, 1), -1.0);
        ASSERT_FLOAT_EQ(u(kNumShells + i), 1e9);
        ASSERT_FLOAT_EQ(l(kNumShells + i), -1e9);
    }
}

//NOLINTNEXTLINE
TEST(LinearCBF, CalculateOurGoalConstraintsLeftOfGoal) {
    FieldDimensions field_dimensions = FieldDimensions::kDefaultDimensions;
    PlayState play_state = PlayState::playing();
    WorldState world_state = initialize_world_state(
        {{
            {{-1.2, 0.0, 0.0}},
            {{-1.0, 4.5, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{-1.0, 4.5, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0}}
        }},
        {0, 0},
        {0, 0}
    );

    Eigen::Matrix<double, LinearCBF::kConstraints, 2> A; //NOLINT(readability-identifier-length, readability-identifier-naming)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> l; //NOLINT(readability-identifier-length)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> u; //NOLINT(readability-identifier-length)

    LinearCBF cbf = LinearCBF(0, 1.0, 1.0 / 30.0);
    cbf.calculate_our_goal_constraints(world_state, play_state, field_dimensions, A, l, u);

    // Right Defense Area Barrier
    ASSERT_FLOAT_EQ(A(LinearCBF::kOurGoalConstraintsIdx, 0), -1);
    ASSERT_FLOAT_EQ(A(LinearCBF::kOurGoalConstraintsIdx, 1), 0);
    ASSERT_FLOAT_EQ(u(LinearCBF::kOurGoalConstraintsIdx), 1e9);
    ASSERT_FLOAT_EQ(l(LinearCBF::kOurGoalConstraintsIdx), -1e9);

    // Left Defense Area Barrier
    ASSERT_FLOAT_EQ(A(LinearCBF::kOurGoalConstraintsIdx + 1, 0), 1);
    ASSERT_FLOAT_EQ(A(LinearCBF::kOurGoalConstraintsIdx + 1, 1), 0);
    ASSERT_FLOAT_EQ(u(LinearCBF::kOurGoalConstraintsIdx + 1), 0.09);
    ASSERT_FLOAT_EQ(l(LinearCBF::kOurGoalConstraintsIdx + 1), -1e9);

    // Bottom Defense Area Barrier
    ASSERT_FLOAT_EQ(A(LinearCBF::kOurGoalConstraintsIdx + 2, 0), 0);
    ASSERT_FLOAT_EQ(A(LinearCBF::kOurGoalConstraintsIdx + 2, 1), 1);
    ASSERT_FLOAT_EQ(u(LinearCBF::kOurGoalConstraintsIdx + 2), 1e9);
    ASSERT_FLOAT_EQ(l(LinearCBF::kOurGoalConstraintsIdx + 2), -1e9);
}

//NOLINTNEXTLINE
TEST(LinearCBF, CalculateOurGoalConstraintsRightOfGoal) {
    FieldDimensions field_dimensions = FieldDimensions::kDefaultDimensions;
    PlayState play_state = PlayState::playing();
    WorldState world_state = initialize_world_state(
        {{
            {{1.2, 0.0, 0.0}},
            {{-1.0, 4.5, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{-1.0, 4.5, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0}}
        }},
        {0, 0},
        {0, 0}
    );

    Eigen::Matrix<double, LinearCBF::kConstraints, 2> A; //NOLINT(readability-identifier-length, readability-identifier-naming)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> l; //NOLINT(readability-identifier-length)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> u; //NOLINT(readability-identifier-length)

    LinearCBF cbf = LinearCBF(0, 1.0, 1.0 / 30.0);
    cbf.calculate_our_goal_constraints(world_state, play_state, field_dimensions, A, l, u);

    // Right Defense Area Barrier
    ASSERT_FLOAT_EQ(A(LinearCBF::kOurGoalConstraintsIdx, 0), -1);
    ASSERT_FLOAT_EQ(A(LinearCBF::kOurGoalConstraintsIdx, 1), 0);
    ASSERT_FLOAT_EQ(u(LinearCBF::kOurGoalConstraintsIdx), 0.09);
    ASSERT_FLOAT_EQ(l(LinearCBF::kOurGoalConstraintsIdx), -1e9);

    // Left Defense Area Barrier
    ASSERT_FLOAT_EQ(A(LinearCBF::kOurGoalConstraintsIdx + 1, 0), 1);
    ASSERT_FLOAT_EQ(A(LinearCBF::kOurGoalConstraintsIdx + 1, 1), 0);
    ASSERT_FLOAT_EQ(u(LinearCBF::kOurGoalConstraintsIdx + 1), 1e9);
    ASSERT_FLOAT_EQ(l(LinearCBF::kOurGoalConstraintsIdx) + 1, -1e9);

    // Bottom Defense Area Barrier
    ASSERT_FLOAT_EQ(A(LinearCBF::kOurGoalConstraintsIdx + 2, 0), 0);
    ASSERT_FLOAT_EQ(A(LinearCBF::kOurGoalConstraintsIdx + 2, 1), 1);
    ASSERT_FLOAT_EQ(u(LinearCBF::kOurGoalConstraintsIdx + 2), 1e9);
    ASSERT_FLOAT_EQ(l(LinearCBF::kOurGoalConstraintsIdx + 2), -1e9);
}

//NOLINTNEXTLINE
TEST(LinearCBF, CalculateOurGoalConstraintsBelowGoal) {
    FieldDimensions field_dimensions = FieldDimensions::kDefaultDimensions;
    PlayState play_state = PlayState::playing();
    WorldState world_state = initialize_world_state(
        {{
            {{0.0, 1.2, 0.0}},
            {{-1.0, 4.5, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{-1.0, 4.5, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0}}
        }},
        {0, 0},
        {0, 0}
    );

    Eigen::Matrix<double, LinearCBF::kConstraints, 2> A; //NOLINT(readability-identifier-length, readability-identifier-naming)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> l; //NOLINT(readability-identifier-length)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> u; //NOLINT(readability-identifier-length)

    LinearCBF cbf = LinearCBF(0, 1.0, 1.0 / 2.0);
    cbf.calculate_our_goal_constraints(world_state, play_state, field_dimensions, A, l, u);

    // Right Defense Area Barrier
    ASSERT_FLOAT_EQ(A(LinearCBF::kOurGoalConstraintsIdx, 0), -1);
    ASSERT_FLOAT_EQ(A(LinearCBF::kOurGoalConstraintsIdx, 1), 0);
    ASSERT_FLOAT_EQ(u(LinearCBF::kOurGoalConstraintsIdx), 1e9);
    ASSERT_FLOAT_EQ(l(LinearCBF::kOurGoalConstraintsIdx), -1e9);

    // Left Defense Area Barrier
    ASSERT_FLOAT_EQ(A(LinearCBF::kOurGoalConstraintsIdx + 1, 0), 1);
    ASSERT_FLOAT_EQ(A(LinearCBF::kOurGoalConstraintsIdx + 1, 1), 0);
    ASSERT_FLOAT_EQ(u(LinearCBF::kOurGoalConstraintsIdx + 1), 1e9);
    ASSERT_FLOAT_EQ(l(LinearCBF::kOurGoalConstraintsIdx + 1), -1e9);

    // Bottom Defense Area Barrier
    ASSERT_FLOAT_EQ(A(LinearCBF::kOurGoalConstraintsIdx + 2, 0), 0);
    ASSERT_FLOAT_EQ(A(LinearCBF::kOurGoalConstraintsIdx + 2, 1), 1);
    ASSERT_FLOAT_EQ(u(LinearCBF::kOurGoalConstraintsIdx + 2), 0.1);
    ASSERT_FLOAT_EQ(l(LinearCBF::kOurGoalConstraintsIdx + 2), -1e9);
}

//NOLINTNEXTLINE
TEST(LinearCBF, CalculateOurGoalConstraintsGoalie) {
    FieldDimensions field_dimensions = FieldDimensions::kDefaultDimensions;
    PlayState play_state = PlayState::playing();
    WorldState world_state = initialize_world_state(
        {{
            {{0.0, 0.5, 0.0}},
            {{1.3, 1.3, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{-1.0, -1.0, 0.0}}
        }},
        {{
            {{0.7, 0.7, 0.0}},
            {{0.1, 0.1, 0.0}}
        }},
        {{
            {{1.0, 1.0, 0.0}},
            {{0.0, 0.0, 0.0}}
        }},
        {0, 0},
        {0, 0}
    );

    Eigen::Matrix<double, LinearCBF::kConstraints, 2> A; //NOLINT(readability-identifier-length, readability-identifier-naming)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> l; //NOLINT(readability-identifier-length)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> u; //NOLINT(readability-identifier-length)

    LinearCBF cbf = LinearCBF(0, 1.0, 1.0 / 2.0, true);
    cbf.calculate_our_goal_constraints(world_state, play_state, field_dimensions, A, l, u);

    // Right Defense Area Barrier
    ASSERT_FLOAT_EQ(A(LinearCBF::kOurGoalConstraintsIdx, 0), -1);
    ASSERT_FLOAT_EQ(A(LinearCBF::kOurGoalConstraintsIdx, 1), 0);
    ASSERT_FLOAT_EQ(u(LinearCBF::kOurGoalConstraintsIdx), 1e9);
    ASSERT_FLOAT_EQ(l(LinearCBF::kOurGoalConstraintsIdx), -1e9);

    // Left Defense Area Barrier
    ASSERT_FLOAT_EQ(A(LinearCBF::kOurGoalConstraintsIdx + 1, 0), 1);
    ASSERT_FLOAT_EQ(A(LinearCBF::kOurGoalConstraintsIdx + 1, 1), 0);
    ASSERT_FLOAT_EQ(u(LinearCBF::kOurGoalConstraintsIdx + 1), 1e9);
    ASSERT_FLOAT_EQ(l(LinearCBF::kOurGoalConstraintsIdx + 1), -1e9);

    // Bottom Defense Area Barrier
    ASSERT_FLOAT_EQ(A(LinearCBF::kOurGoalConstraintsIdx + 2, 0), 0);
    ASSERT_FLOAT_EQ(A(LinearCBF::kOurGoalConstraintsIdx + 2, 1), 1);
    ASSERT_FLOAT_EQ(u(LinearCBF::kOurGoalConstraintsIdx + 2), 1e9);
    ASSERT_FLOAT_EQ(l(LinearCBF::kOurGoalConstraintsIdx + 2), -1e9);
}

//NOLINTNEXTLINE
TEST(LinearCBF, CalculateTheirGoalConstraintsRight) {
    FieldDimensions field_dimensions = FieldDimensions::kDefaultDimensions;
    PlayState play_state = PlayState::playing();
    WorldState world_state = initialize_world_state(
        {{
            {{1.2, 9.0, 0.0}},
            {{-1.0, 4.5, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{-1.0, 4.5, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0}}
        }},
        {0, 0},
        {0, 0}
    );

    Eigen::Matrix<double, LinearCBF::kConstraints, 2> A; //NOLINT(readability-identifier-length, readability-identifier-naming)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> l; //NOLINT(readability-identifier-length)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> u; //NOLINT(readability-identifier-length)

    LinearCBF cbf = LinearCBF(0, 1.0, 1.0 / 30.0);
    cbf.calculate_their_goal_constraints(world_state, play_state, field_dimensions, A, l, u);

    // Right Defense Area Barrier
    ASSERT_FLOAT_EQ(A(LinearCBF::kTheirGoalConstraintsIdx, 0), -1);
    ASSERT_FLOAT_EQ(A(LinearCBF::kTheirGoalConstraintsIdx, 1), 0);
    ASSERT_FLOAT_EQ(u(LinearCBF::kTheirGoalConstraintsIdx), 0.09);
    ASSERT_FLOAT_EQ(l(LinearCBF::kTheirGoalConstraintsIdx), -1e9);

    // Left Defense Area Barrier
    ASSERT_FLOAT_EQ(A(LinearCBF::kTheirGoalConstraintsIdx + 1, 0), 1);
    ASSERT_FLOAT_EQ(A(LinearCBF::kTheirGoalConstraintsIdx + 1, 1), 0);
    ASSERT_FLOAT_EQ(u(LinearCBF::kTheirGoalConstraintsIdx + 1), 1e9);
    ASSERT_FLOAT_EQ(l(LinearCBF::kTheirGoalConstraintsIdx) + 1, -1e9);

    // Bottom Defense Area Barrier
    ASSERT_FLOAT_EQ(A(LinearCBF::kTheirGoalConstraintsIdx + 2, 0), 0);
    ASSERT_FLOAT_EQ(A(LinearCBF::kTheirGoalConstraintsIdx + 2, 1), -1);
    ASSERT_FLOAT_EQ(u(LinearCBF::kTheirGoalConstraintsIdx + 2), 1e9);
    ASSERT_FLOAT_EQ(l(LinearCBF::kTheirGoalConstraintsIdx + 2), -1e9);
}

//NOLINTNEXTLINE
TEST(LinearCBF, CalculateTheirGoalConstraintsLeft) {
    FieldDimensions field_dimensions = FieldDimensions::kDefaultDimensions;
    PlayState play_state = PlayState::playing();
    WorldState world_state = initialize_world_state(
        {{
            {{-1.2, 9.0, 0.0}},
            {{-1.0, 4.5, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{-1.0, 4.5, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0}}
        }},
        {0, 0},
        {0, 0}
    );

    Eigen::Matrix<double, LinearCBF::kConstraints, 2> A; //NOLINT(readability-identifier-length, readability-identifier-naming)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> l; //NOLINT(readability-identifier-length)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> u; //NOLINT(readability-identifier-length)

    LinearCBF cbf = LinearCBF(0, 1.0, 1.0 / 30.0);
    cbf.calculate_their_goal_constraints(world_state, play_state, field_dimensions, A, l, u);

    // Right Defense Area Barrier
    ASSERT_FLOAT_EQ(A(LinearCBF::kTheirGoalConstraintsIdx, 0), -1);
    ASSERT_FLOAT_EQ(A(LinearCBF::kTheirGoalConstraintsIdx, 1), 0);
    ASSERT_FLOAT_EQ(u(LinearCBF::kTheirGoalConstraintsIdx), 1e9);
    ASSERT_FLOAT_EQ(l(LinearCBF::kTheirGoalConstraintsIdx), -1e9);

    // Left Defense Area Barrier
    ASSERT_FLOAT_EQ(A(LinearCBF::kTheirGoalConstraintsIdx + 1, 0), 1);
    ASSERT_FLOAT_EQ(A(LinearCBF::kTheirGoalConstraintsIdx + 1, 1), 0);
    ASSERT_FLOAT_EQ(u(LinearCBF::kTheirGoalConstraintsIdx + 1), 0.09);
    ASSERT_FLOAT_EQ(l(LinearCBF::kTheirGoalConstraintsIdx) + 1, -1e9);

    // Bottom Defense Area Barrier
    ASSERT_FLOAT_EQ(A(LinearCBF::kTheirGoalConstraintsIdx + 2, 0), 0);
    ASSERT_FLOAT_EQ(A(LinearCBF::kTheirGoalConstraintsIdx + 2, 1), -1);
    ASSERT_FLOAT_EQ(u(LinearCBF::kTheirGoalConstraintsIdx + 2), 1e9);
    ASSERT_FLOAT_EQ(l(LinearCBF::kTheirGoalConstraintsIdx + 2), -1e9);
}

//NOLINTNEXTLINE
TEST(LinearCBF, CalculateTheirGoalConstraintsBottom) {
    FieldDimensions field_dimensions = FieldDimensions::kDefaultDimensions;
    PlayState play_state = PlayState::playing();
    WorldState world_state = initialize_world_state(
        {{
            {{0.0, 7.8, 0.0}},
            {{-1.0, 4.5, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{-1.0, 4.5, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0}}
        }},
        {0, 0},
        {0, 0}
    );

    Eigen::Matrix<double, LinearCBF::kConstraints, 2> A; //NOLINT(readability-identifier-length, readability-identifier-naming)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> l; //NOLINT(readability-identifier-length)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> u; //NOLINT(readability-identifier-length)

    LinearCBF cbf = LinearCBF(0, 1.0, 1.0 / 30.0);
    cbf.calculate_their_goal_constraints(world_state, play_state, field_dimensions, A, l, u);

    // Right Defense Area Barrier
    ASSERT_FLOAT_EQ(A(LinearCBF::kTheirGoalConstraintsIdx, 0), -1);
    ASSERT_FLOAT_EQ(A(LinearCBF::kTheirGoalConstraintsIdx, 1), 0);
    ASSERT_FLOAT_EQ(u(LinearCBF::kTheirGoalConstraintsIdx), 1e9);
    ASSERT_FLOAT_EQ(l(LinearCBF::kTheirGoalConstraintsIdx), -1e9);

    // Left Defense Area Barrier
    ASSERT_FLOAT_EQ(A(LinearCBF::kTheirGoalConstraintsIdx + 1, 0), 1);
    ASSERT_FLOAT_EQ(A(LinearCBF::kTheirGoalConstraintsIdx + 1, 1), 0);
    ASSERT_FLOAT_EQ(u(LinearCBF::kTheirGoalConstraintsIdx + 1), 1e9);
    ASSERT_FLOAT_EQ(l(LinearCBF::kTheirGoalConstraintsIdx) + 1, -1e9);

    // Bottom Defense Area Barrier
    ASSERT_FLOAT_EQ(A(LinearCBF::kTheirGoalConstraintsIdx + 2, 0), 0);
    ASSERT_FLOAT_EQ(A(LinearCBF::kTheirGoalConstraintsIdx + 2, 1), -1);
    ASSERT_FLOAT_EQ(u(LinearCBF::kTheirGoalConstraintsIdx + 2), 0.1);
    ASSERT_FLOAT_EQ(l(LinearCBF::kTheirGoalConstraintsIdx + 2), -1e9);
}

//NOLINTNEXTLINE
TEST(LinearCBF, CalculateBallPositionConstraintsStop) {
    FieldDimensions field_dimensions = FieldDimensions::kDefaultDimensions;
    PlayState play_state = PlayState::stop();
    WorldState world_state = initialize_world_state(
        {{
            {{0.7, 0.0, 0.0}},
            {{1.3, 1.3, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{-1.0, -1.0, 0.0}}
        }},
        {{
            {{0.7, 0.7, 0.0}},
            {{0.1, 0.1, 0.0}}
        }},
        {{
            {{1.0, 1.0, 0.0}},
            {{0.0, 0.0, 0.0}}
        }},
        {0, 0},
        {1.0, 0}
    );

    Eigen::Matrix<double, LinearCBF::kConstraints, 2> A; //NOLINT(readability-identifier-length, readability-identifier-naming)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> l; //NOLINT(readability-identifier-length)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> u; //NOLINT(readability-identifier-length)

    LinearCBF cbf = LinearCBF(0, 1.0, 1.0);
    cbf.calculate_ball_position_constraints(world_state, play_state, A, l, u);

    ASSERT_FLOAT_EQ(A(LinearCBF::kBallConstraintIdx, 0), -0.7);
    ASSERT_FLOAT_EQ(A(LinearCBF::kBallConstraintIdx, 1), 0.0);
    ASSERT_FLOAT_EQ(u(LinearCBF::kBallConstraintIdx), -0.7 + 0.7 * (0.1));
    ASSERT_FLOAT_EQ(l(LinearCBF::kBallConstraintIdx), -1e9);
}

//NOLINTNEXTLINE
TEST(LinearCBF, CalculateBallPositionConstraintsPlay) {
    FieldDimensions field_dimensions = FieldDimensions::kDefaultDimensions;
    PlayState play_state = PlayState::playing();
    WorldState world_state = initialize_world_state(
        {{
            {{0.6, 0.0, 0.0}},
            {{1.3, 1.3, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{-1.0, -1.0, 0.0}}
        }},
        {{
            {{0.7, 0.7, 0.0}},
            {{0.1, 0.1, 0.0}}
        }},
        {{
            {{1.0, 1.0, 0.0}},
            {{0.0, 0.0, 0.0}}
        }},
        {0, 0},
        {1.0, 0}
    );

    Eigen::Matrix<double, LinearCBF::kConstraints, 2> A; //NOLINT(readability-identifier-length, readability-identifier-naming)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> l; //NOLINT(readability-identifier-length)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> u; //NOLINT(readability-identifier-length)

    LinearCBF cbf = LinearCBF(0, 1.0, 1.0);
    cbf.calculate_ball_position_constraints(world_state, play_state, A, l, u);

    ASSERT_FLOAT_EQ(A(LinearCBF::kBallConstraintIdx, 0), -0.6);
    ASSERT_FLOAT_EQ(A(LinearCBF::kBallConstraintIdx, 1), 0.0);
    ASSERT_FLOAT_EQ(u(LinearCBF::kBallConstraintIdx), 1e9);
    ASSERT_FLOAT_EQ(l(LinearCBF::kBallConstraintIdx), -1e9);
}

//NOLINTNEXTLINE
TEST(LinearCBF, CalculateBallPlacementConstraintsGoalLocation) {
    FieldDimensions field_dimensions = FieldDimensions::kDefaultDimensions;
    PlayState play_state = PlayState::ball_placement(false, {1.0, 0.0});
    WorldState world_state = initialize_world_state(
        {{
            {{1.7, 0.0, 0.0}},
            {{1.3, 1.3, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{-1.0, -1.0, 0.0}}
        }},
        {{
            {{0.7, 0.7, 0.0}},
            {{0.1, 0.1, 0.0}}
        }},
        {{
            {{1.0, 1.0, 0.0}},
            {{0.0, 0.0, 0.0}}
        }},
        {0, 0},
        {1.0, 0}
    );

    Eigen::Matrix<double, LinearCBF::kConstraints, 2> A; //NOLINT(readability-identifier-length, readability-identifier-naming)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> l; //NOLINT(readability-identifier-length)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> u; //NOLINT(readability-identifier-length)

    LinearCBF cbf = LinearCBF(0, 1.0, 1.0);
    cbf.calculate_ball_placement_constraints(world_state, play_state, A, l ,u);

    ASSERT_FLOAT_EQ(A(LinearCBF::kBallPlacementConstraintIdx, 0), -0.7);
    ASSERT_FLOAT_EQ(A(LinearCBF::kBallPlacementConstraintIdx, 1), 0);
    ASSERT_FLOAT_EQ(u(LinearCBF::kBallPlacementConstraintIdx), 0.07);
    ASSERT_FLOAT_EQ(l(LinearCBF::kBallPlacementConstraintIdx), -1e9);

    ASSERT_FLOAT_EQ(A(LinearCBF::kBallPlacementConstraintIdx + 1, 0), 0);
    ASSERT_FLOAT_EQ(A(LinearCBF::kBallPlacementConstraintIdx + 1, 1), -1);
    ASSERT_FLOAT_EQ(u(LinearCBF::kBallPlacementConstraintIdx + 1), 1e9);
    ASSERT_FLOAT_EQ(l(LinearCBF::kBallPlacementConstraintIdx + 1), -1e9);
}

//NOLINTNEXTLINE
TEST(LinearCBF, CalculateBallPlacementConstraintsBallPlacementLine) {
    FieldDimensions field_dimensions = FieldDimensions::kDefaultDimensions;
    PlayState play_state = PlayState::ball_placement(false, {6.0, 0.0});
    WorldState world_state = initialize_world_state(
        {{
            {{3.0, 0.7, 0.0}},
            {{1.3, 1.3, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{-1.0, -1.0, 0.0}}
        }},
        {{
            {{0.7, 0.7, 0.0}},
            {{0.1, 0.1, 0.0}}
        }},
        {{
            {{1.0, 1.0, 0.0}},
            {{0.0, 0.0, 0.0}}
        }},
        {0, 0},
        {1.0, 0}
    );

    Eigen::Matrix<double, LinearCBF::kConstraints, 2> A; //NOLINT(readability-identifier-length, readability-identifier-naming)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> l; //NOLINT(readability-identifier-length)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> u; //NOLINT(readability-identifier-length)

    LinearCBF cbf = LinearCBF(0, 1.0, 1.0);
    cbf.calculate_ball_placement_constraints(world_state, play_state, A, l ,u);

    ASSERT_FLOAT_EQ(A(LinearCBF::kBallPlacementConstraintIdx, 0), 3);
    ASSERT_FLOAT_EQ(A(LinearCBF::kBallPlacementConstraintIdx, 1), -0.7);
    ASSERT_FLOAT_EQ(u(LinearCBF::kBallPlacementConstraintIdx), 1e9);
    ASSERT_FLOAT_EQ(l(LinearCBF::kBallPlacementConstraintIdx), -1e9);

    ASSERT_FLOAT_EQ(A(LinearCBF::kBallPlacementConstraintIdx + 1, 0), 0);
    ASSERT_FLOAT_EQ(A(LinearCBF::kBallPlacementConstraintIdx + 1, 1), 1);
    ASSERT_FLOAT_EQ(u(LinearCBF::kBallPlacementConstraintIdx + 1), 0.1);
    ASSERT_FLOAT_EQ(l(LinearCBF::kBallPlacementConstraintIdx + 1), -1e9);
}

//NOLINTNEXTLINE
TEST(LinearCBF, CalculateBallPlacementConstraintsNotBallPlacement) {
    FieldDimensions field_dimensions = FieldDimensions::kDefaultDimensions;
    PlayState play_state = PlayState::playing();
    WorldState world_state = initialize_world_state(
        {{
            {{0.7, 0.0, 0.0}},
            {{1.3, 1.3, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{-1.0, -1.0, 0.0}}
        }},
        {{
            {{0.7, 0.7, 0.0}},
            {{0.1, 0.1, 0.0}}
        }},
        {{
            {{1.0, 1.0, 0.0}},
            {{0.0, 0.0, 0.0}}
        }},
        {0, 0},
        {1.0, 0}
    );

    Eigen::Matrix<double, LinearCBF::kConstraints, 2> A; //NOLINT(readability-identifier-length, readability-identifier-naming)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> l; //NOLINT(readability-identifier-length)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> u; //NOLINT(readability-identifier-length)

    LinearCBF cbf = LinearCBF(0, 1.0, 1.0 / 30.0);
    cbf.calculate_ball_placement_constraints(world_state, play_state, A, l, u);

    ASSERT_FLOAT_EQ(A(LinearCBF::kBallPlacementConstraintIdx, 0), -1.0);
    ASSERT_FLOAT_EQ(A(LinearCBF::kBallPlacementConstraintIdx, 1), -1.0);
    ASSERT_FLOAT_EQ(u(LinearCBF::kBallPlacementConstraintIdx), 1e9);
    ASSERT_FLOAT_EQ(l(LinearCBF::kBallPlacementConstraintIdx), -1e9);

    ASSERT_FLOAT_EQ(A(LinearCBF::kBallPlacementConstraintIdx + 1, 0), -1.0);
    ASSERT_FLOAT_EQ(A(LinearCBF::kBallPlacementConstraintIdx + 1, 1), -1.0);
    ASSERT_FLOAT_EQ(u(LinearCBF::kBallPlacementConstraintIdx + 1), 1e9);
    ASSERT_FLOAT_EQ(l(LinearCBF::kBallPlacementConstraintIdx + 1), -1e9);
}

//NOLINTNEXTLINE
TEST(LinearCBF, CalculateVelocityConstraintsPlay) {
    PlayState play_state = PlayState::playing();

    Eigen::Matrix<double, LinearCBF::kConstraints, 2> A; //NOLINT(readability-identifier-length, readability-identifier-naming)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> l; //NOLINT(readability-identifier-length)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> u; //NOLINT(readability-identifier-length)

    LinearCBF::calculate_velocity_constraints(play_state, A, l, u);

    // L2 Maximum Speed in the x-direction
    ASSERT_FLOAT_EQ(A(LinearCBF::kVelocityConstraintIdx, 0), 1);
    ASSERT_FLOAT_EQ(A(LinearCBF::kVelocityConstraintIdx, 1), 0);
    ASSERT_FLOAT_EQ(l(LinearCBF::kVelocityConstraintIdx), -4.0);
    ASSERT_FLOAT_EQ(u(LinearCBF::kVelocityConstraintIdx), 4.0);

    // L2 Maximum Speed in the y-direction
    ASSERT_FLOAT_EQ(A(LinearCBF::kVelocityConstraintIdx + 1, 0), 0);
    ASSERT_FLOAT_EQ(A(LinearCBF::kVelocityConstraintIdx + 1, 1), 1);
    ASSERT_FLOAT_EQ(l(LinearCBF::kVelocityConstraintIdx + 1), -4.0);
    ASSERT_FLOAT_EQ(u(LinearCBF::kVelocityConstraintIdx + 1), 4.0);

    // 8-sided approximation
    ASSERT_FLOAT_EQ(A(LinearCBF::kVelocityConstraintIdx + 2, 0), 1 / sqrt(2));
    ASSERT_FLOAT_EQ(A(LinearCBF::kVelocityConstraintIdx + 2, 1), 1 / sqrt(2));
    ASSERT_FLOAT_EQ(l(LinearCBF::kVelocityConstraintIdx + 2), -4.0);
    ASSERT_FLOAT_EQ(u(LinearCBF::kVelocityConstraintIdx + 2), 4.0);

    ASSERT_FLOAT_EQ(A(LinearCBF::kVelocityConstraintIdx + 3, 0), 1 / sqrt(2));
    ASSERT_FLOAT_EQ(A(LinearCBF::kVelocityConstraintIdx + 3, 1), -1 / sqrt(2));
    ASSERT_FLOAT_EQ(l(LinearCBF::kVelocityConstraintIdx + 3), -4.0);
    ASSERT_FLOAT_EQ(u(LinearCBF::kVelocityConstraintIdx + 3), 4.0);
}

//NOLINTNEXTLINE
TEST(LinearCBF, CalculateVelocityConstraintsStop) {
    PlayState play_state = PlayState::stop();

    Eigen::Matrix<double, LinearCBF::kConstraints, 2> A; //NOLINT(readability-identifier-length, readability-identifier-naming)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> l; //NOLINT(readability-identifier-length)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> u; //NOLINT(readability-identifier-length)

    LinearCBF::calculate_velocity_constraints(play_state, A, l, u);

    // L2 Maximum Speed in the x-direction
    ASSERT_FLOAT_EQ(A(LinearCBF::kVelocityConstraintIdx, 0), 1);
    ASSERT_FLOAT_EQ(A(LinearCBF::kVelocityConstraintIdx, 1), 0);
    ASSERT_FLOAT_EQ(l(LinearCBF::kVelocityConstraintIdx), -1.5);
    ASSERT_FLOAT_EQ(u(LinearCBF::kVelocityConstraintIdx), 1.5);

    // L2 Maximum Speed in the y-direction
    ASSERT_FLOAT_EQ(A(LinearCBF::kVelocityConstraintIdx + 1, 0), 0);
    ASSERT_FLOAT_EQ(A(LinearCBF::kVelocityConstraintIdx + 1, 1), 1);
    ASSERT_FLOAT_EQ(l(LinearCBF::kVelocityConstraintIdx + 1), -1.5);
    ASSERT_FLOAT_EQ(u(LinearCBF::kVelocityConstraintIdx + 1), 1.5);

    // 8-sided approximation
    ASSERT_FLOAT_EQ(A(LinearCBF::kVelocityConstraintIdx + 2, 0), 1 / sqrt(2));
    ASSERT_FLOAT_EQ(A(LinearCBF::kVelocityConstraintIdx + 2, 1), 1 / sqrt(2));
    ASSERT_FLOAT_EQ(l(LinearCBF::kVelocityConstraintIdx + 2), -1.5);
    ASSERT_FLOAT_EQ(u(LinearCBF::kVelocityConstraintIdx + 2), 1.5);

    ASSERT_FLOAT_EQ(A(LinearCBF::kVelocityConstraintIdx + 3, 0), 1 / sqrt(2));
    ASSERT_FLOAT_EQ(A(LinearCBF::kVelocityConstraintIdx + 3, 1), -1 / sqrt(2));
    ASSERT_FLOAT_EQ(l(LinearCBF::kVelocityConstraintIdx + 3), -1.5);
    ASSERT_FLOAT_EQ(u(LinearCBF::kVelocityConstraintIdx + 3), 1.5);
}

//NOLINTNEXTLINE
TEST(LinearCBF, ApplyCBF) {
    FieldDimensions field_dimensions = FieldDimensions::kDefaultDimensions;
    PlayState play_state = PlayState::playing();
    WorldState world_state = initialize_world_state(
        {{
            {{0.0, 4.5, 0.0}},
            {{-2.0, 1.3, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{-1.0, -1.0, 0.0}}
        }},
        {{
            {{0.5, 4.7, 0.0}},
            {{0.1, 0.1, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0}}
        }},
        {0, 0},
        {0, 0}
    );

    Eigen::Matrix<double, LinearCBF::kConstraints, 2> A; //NOLINT(readability-identifier-length, readability-identifier-naming)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> l; //NOLINT(readability-identifier-length)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> u; //NOLINT(readability-identifier-length)

    LinearCBF cbf = LinearCBF(0, 1.0, 1.0 / 30.0);
    rj_geometry::Twist actual_control = cbf.apply(
        world_state,
        play_state,
        field_dimensions,
        {1.0, 0.0, 1.0}
    );

    ASSERT_FLOAT_EQ(actual_control.linear().x(), 0.99999434);
    ASSERT_FLOAT_EQ(actual_control.linear().y(), -7.1493073e-06);
    ASSERT_FLOAT_EQ(actual_control.angular(), 1.0);
}

//NOLINTNEXTLINE
TEST(LinearCBF, ApplyCBFAlreadyCompletelySafe) {
    FieldDimensions field_dimensions = FieldDimensions::kDefaultDimensions;
    PlayState play_state = PlayState::playing();
    WorldState world_state = initialize_world_state(
        {{
            {{0.0, 4.5, 0.0}},
            {{-2.0, 1.3, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{-1.0, -1.0, 0.0}}
        }},
        {},
        {},
        {0, 0},
        {0, 0}
    );

    Eigen::Matrix<double, LinearCBF::kConstraints, 2> A; //NOLINT(readability-identifier-length, readability-identifier-naming)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> l; //NOLINT(readability-identifier-length)
    Eigen::Matrix<double, LinearCBF::kConstraints, 1> u; //NOLINT(readability-identifier-length)

    LinearCBF cbf = LinearCBF(0, 1.0, 1.0 / 30.0);
    rj_geometry::Twist actual_control = cbf.apply(
        world_state,
        play_state,
        field_dimensions,
        {1.0, 0.0, -1.0}
    );

    ASSERT_FLOAT_EQ(actual_control.linear().x(), 0.99999213);
    ASSERT_FLOAT_EQ(actual_control.linear().y(), -9.362986e-06);
    ASSERT_FLOAT_EQ(actual_control.angular(), -1.0);
}

} // namespace control