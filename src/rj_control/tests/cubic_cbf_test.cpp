#include <gtest/gtest.h>

#include <rj_geometry/pose.hpp>
#include <rj_common/world_state.hpp>
#include <rj_common/game_state.hpp>
#include <rj_common/field_dimensions.hpp>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <rj_control/barriers/cubic_cbf.hpp>

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
TEST(CubicCBF, ApplyCBF) {
    FieldDimensions field_dimensions = FieldDimensions::kDefaultDimensions;
    PlayState play_state = PlayState::playing();
    WorldState world_state = initialize_world_state(
        {{
            {{0.0, 0.0, 0.0}},
            {{-4.0, 4.0, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0}}
        }},
        {{
            {{0.3, 0.0, 0.0}},
            {{0.6, 0.0, 0.0}}
        }},
        {{
            {{0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0}}
        }},
        {0, 0},
        {0, 0}
    );

    CubicCBF cbf = CubicCBF(0, 1.0, 1.0 / 30.0, false);
    rj_geometry::Twist nominal_control = {1.0, 0.0, 0.0};
    rj_geometry::Twist updated_control = cbf.apply(
        world_state,
        play_state,
        field_dimensions,
        nominal_control
    );

    ASSERT_FLOAT_EQ(updated_control.linear().x(), 8.5134358e-05);
    ASSERT_FLOAT_EQ(updated_control.linear().y(), 0.00029715925);
    ASSERT_FLOAT_EQ(updated_control.angular(), 0.0);
}

} // namespace control