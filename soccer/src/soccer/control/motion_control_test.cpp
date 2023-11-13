#include <gtest/gtest.h>

#include "motion_control_node.hpp"
#include "game_state.hpp"

namespace control::testing {

using planning::RobotInstant;
using planning::Trajectory;
using rj_geometry::Point;
using rj_geometry::Pose;
using rj_geometry::Twist;

class MotionControlTest : public ::testing::Test {
public:
    void SetUp() override {
        rclcpp::init(0, {});
        node_ = std::make_shared<rclcpp::Node>("test_motion_control");
        control_ = std::make_unique<MotionControl>(0, node_.get());
    }

    void TearDown() override { rclcpp::shutdown(); }

protected:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<MotionControl> control_;

    void run(RobotState state, const Trajectory& trajectory, PlayState::State play_state,
             bool is_joystick_controlled, MotionSetpoint* setpoint) {
        control_->run(state, trajectory, play_state, is_joystick_controlled, setpoint);
    }
};

RobotState make_initial_state() {
    return RobotState{Pose(1.0, 1.0, 0.0), Twist::zero(), RJ::Time::clock::from_time_t(0), true};
}

Trajectory make_trajectory() {
    RobotState initial = make_initial_state();
    RobotInstant initial_instant{initial.pose, initial.velocity, initial.timestamp};
    RobotInstant final_instant{initial.pose + Pose(1.0, 0.0, 0.0), Twist::zero(),
                               initial.timestamp + RJ::Seconds(1.0)};
    return Trajectory({initial_instant, final_instant});
}

// Test when we're in halt, and make sure the controller gives zero output.
TEST_F(MotionControlTest, halt_zero_output) {
    RobotState state = make_initial_state();
    Trajectory trajectory = make_trajectory();
    MotionSetpoint setpoint;

    // Evaluate at 0.5 seconds into the trajectory.
    state.timestamp = state.timestamp + RJ::Seconds(0.5);
    run(state, trajectory, PlayState::Halt, false, &setpoint);

    EXPECT_EQ(setpoint.xvelocity, 0.0);
    EXPECT_EQ(setpoint.yvelocity, 0.0);
    EXPECT_EQ(setpoint.avelocity, 0.0);
}

// Test when we're in running, and make sure the controller gives some nonzero output.
TEST_F(MotionControlTest, running_nonzero_output) {
    RobotState state = make_initial_state();
    Trajectory trajectory = make_trajectory();
    MotionSetpoint setpoint;

    // Evaluate at 0.5 seconds into the trajectory.
    state.timestamp = state.timestamp + RJ::Seconds(0.5);
    run(state, trajectory, PlayState::Playing, false, &setpoint);

    EXPECT_GT(Point(setpoint.xvelocity, setpoint.yvelocity).mag(), 0.1);
}

TEST_F(MotionControlTest, running_after_trajectory) {
    RobotState state = make_initial_state();
    Trajectory trajectory = make_trajectory();
    MotionSetpoint setpoint;

    // Evaluate at 0.5 seconds into the trajectory.
    state.timestamp = state.timestamp + RJ::Seconds(1.5);
    run(state, trajectory, PlayState::Playing, false, &setpoint);

    EXPECT_GT(Point(setpoint.xvelocity, setpoint.yvelocity).mag(), 0.1);
}

TEST_F(MotionControlTest, running_empty_trajectory) {
    RobotState state = make_initial_state();
    Trajectory trajectory;
    MotionSetpoint setpoint;

    // Evaluate at 0.5 seconds into the trajectory.
    state.timestamp = state.timestamp;
    run(state, trajectory, PlayState::Playing, false, &setpoint);

    EXPECT_NEAR(Point(setpoint.xvelocity, setpoint.yvelocity).mag(), 0.0, 1e-6);
}

// Make sure the coordinate transformations are correct
TEST_F(MotionControlTest, coordinate_transform_body_y) {
    RobotState state = make_initial_state();
    Trajectory trajectory = make_trajectory();
    MotionSetpoint setpoint;

    // This trajectory is in world x. If heading is 0, this corresponds to the positive y axis.
    state.timestamp = state.timestamp + RJ::Seconds(0.5);
    run(state, trajectory, PlayState::Playing, false, &setpoint);

    EXPECT_GT(setpoint.yvelocity, 0.1);
    EXPECT_NEAR(setpoint.xvelocity, 0.0, 1e-6);
    EXPECT_NEAR(setpoint.avelocity, 0.0, 1e-6);
}

// Make sure the coordinate transformations are correct
TEST_F(MotionControlTest, coordinate_transform_body_x) {
    RobotState state = make_initial_state();
    Trajectory trajectory = make_trajectory();
    MotionSetpoint setpoint;

    state.pose.heading() = M_PI_2;
    trajectory.instant_at(0).heading() = M_PI_2;
    trajectory.instant_at(1).heading() = M_PI_2;

    // This trajectory is in world x. If heading is 0, this corresponds to the positive y axis.
    state.timestamp = state.timestamp + RJ::Seconds(0.5);
    run(state, trajectory, PlayState::Playing, false, &setpoint);

    // X body velocity
    EXPECT_GT(setpoint.xvelocity, 0.1);
    EXPECT_NEAR(setpoint.yvelocity, 0.0, 1e-6);
    EXPECT_NEAR(setpoint.avelocity, 0.0, 1e-6);
}

}  // namespace control::testing
