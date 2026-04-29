#include <gtest/gtest.h>

#include <rj_msgs/msg/manipulator_setpoint.hpp>
#include <rj_msgs/msg/motion_setpoint.hpp>
#include <rj_msgs/msg/robot_status.hpp>

#include "rj_base_station/messages.hpp"

namespace rtp::testing {

TEST(MessagePacking, ControlMessageFromRos) {
    rj_msgs::msg::MotionSetpoint motion_setpoint;
    motion_setpoint.velocity_x_mps = 1.0;
    motion_setpoint.velocity_y_mps = -1.0;
    motion_setpoint.velocity_z_radps = 3.14;
    auto motion_setpoint_ptr = std::make_shared<rj_msgs::msg::MotionSetpoint>(motion_setpoint);

    rj_msgs::msg::ManipulatorSetpoint manipulator_setpoint;
    manipulator_setpoint.dribbler_speed = 0.5;
    manipulator_setpoint.kick_speed = 0.75;
    auto manipulator_setpoint_ptr =
        std::make_shared<rj_msgs::msg::ManipulatorSetpoint>(manipulator_setpoint);

    rtp::ControlMessage message =
        rtp::ControlMessage::from_ros(1, true, motion_setpoint_ptr, manipulator_setpoint_ptr);

    EXPECT_EQ(message.team, 0);
    EXPECT_EQ(message.robot_id, 1);
    EXPECT_EQ(message.shoot_mode, 0);
    EXPECT_EQ(message.trigger_mode, 0);
    EXPECT_NEAR(message.body_x,
                static_cast<int16_t>(motion_setpoint.velocity_x_mps *
                                     rtp::ControlMessage::kVelocityScaleFactor),
                1);
    EXPECT_NEAR(message.body_y,
                static_cast<int16_t>(motion_setpoint.velocity_y_mps *
                                     rtp::ControlMessage::kVelocityScaleFactor),
                1);
    EXPECT_NEAR(message.body_w,
                static_cast<int16_t>(motion_setpoint.velocity_z_radps *
                                     rtp::ControlMessage::kVelocityScaleFactor),
                1);
    EXPECT_NEAR(message.dribbler_speed,
                static_cast<int8_t>(manipulator_setpoint.dribbler_speed * 100), 1);
    EXPECT_NEAR(message.kick_strength, static_cast<uint8_t>(manipulator_setpoint.kick_speed * 100),
                1);
}

TEST(MessagePacking, RobotStatusToRos) {
    rtp::RobotStatusMessage robot_status = {
        .team = 0,
        .robot_id = 1,
        .ball_sense = 1,
        .kicker_status = 1,
        .kicker_healthy = 1,
        .battery_voltage = 50,
        .unused = 0,
        .fpga_status = 1,
        .motor_errors = 0b11111,
    };

    rj_msgs::msg::RobotStatus ros_status = robot_status.to_ros();
    EXPECT_EQ(ros_status.robot_id, robot_status.robot_id);
    EXPECT_EQ(ros_status.kicker_healthy, robot_status.kicker_healthy);
    EXPECT_EQ(ros_status.has_ball_sense, robot_status.ball_sense);
    EXPECT_EQ(ros_status.battery_voltage, robot_status.battery_voltage);
}

}  // namespace rtp::testing
