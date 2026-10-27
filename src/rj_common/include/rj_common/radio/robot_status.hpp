#pragma once

#include <array>
#include <optional>

#include <rclcpp/type_adapter.hpp>
#include <rj_common/time.hpp>
#include <rj_geometry/pose.hpp>
#include <rj_msgs/msg/robot_status.hpp>

/**
 * \brief Struct representing data returned from the robot, possibly including
 * state updates from an on-board Kalman filter.
 */
struct RobotStatus {
    RJ::Time timestamp;

    int shell_id = 0;

    enum class HardwareVersion { kUnknown, kFleet2018, kSimulated };

    HardwareVersion version = HardwareVersion::kFleet2018;

    std::optional<rj_geometry::Twist> twist_estimate;
    std::optional<rj_geometry::Pose> pose_estimate;

    double battery_voltage = 0.0;
    double kicker_voltage = 0.0;

    bool has_ball = false;

    enum class KickerState { kFailed, kCharging, kCharged };

    KickerState kicker = KickerState::kFailed;

    std::array<bool, 5> motors_healthy{};

    bool fpga_healthy = false;
};

template<>
struct rclcpp::TypeAdapter<RobotStatus, rj_msgs::msg::RobotStatus> {
    using is_specialized = std::true_type;
    using custom_type = RobotStatus;
    using ros_message_type = rj_msgs::msg::RobotStatus;
    static void convert_to_ros_message(const RobotStatus& status, rj_msgs::msg::RobotStatus& msg) {
        rj_convert::convert_to_ros(status.timestamp, &msg.timestamp);
        msg.robot_id = status.shell_id;
        msg.battery_voltage = status.battery_voltage;
        msg.motor_errors = status.motors_healthy;
        msg.has_ball_sense = status.has_ball;
        msg.kicker_charged = status.kicker == RobotStatus::KickerState::kCharged;
        msg.kicker_healthy = status.kicker != RobotStatus::KickerState::kFailed;
        msg.fpga_error = !status.fpga_healthy;
    }

    static void convert_to_custom(const rj_msgs::msg::RobotStatus& msg, RobotStatus& status) {
        rj_convert::convert_from_ros(msg.timestamp, &status.timestamp);
        status.shell_id = msg.robot_id;
        status.battery_voltage = msg.battery_voltage;
        status.motors_healthy = msg.motor_errors;
        status.has_ball = msg.has_ball_sense;
        if (!msg.kicker_healthy) {
            status.kicker = RobotStatus::KickerState::kFailed;
        } else if (msg.kicker_charged) {
            status.kicker = RobotStatus::KickerState::kCharged;
        } else {
            status.kicker = RobotStatus::KickerState::kCharging;
        }
        status.fpga_healthy = !msg.fpga_error;
    }
};
