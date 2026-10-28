#pragma once

#include <memory>


#include <rj_geometry/shape_set.hpp>
#include <rj_msgs/msg/robot_intent.hpp>

#include "rj_common/planning/motion_command.hpp"
#include "rj_common/planning/rotation_command.hpp"

struct RobotIntent {
    int8_t robot_id = 0;

    using Msg = rj_msgs::msg::RobotIntent;
    using SharedPtr = std::shared_ptr<RobotIntent>;
    using ConstSharedPtr = std::shared_ptr<const RobotIntent>;
    enum ShootMode { KICK, CHIP };
    enum TriggerMode { STAND_DOWN = 0, IMMEDIATE, ON_BREAK_BEAM, AT_END };
    enum DribblerMode { OFF = 0, ON, DEFAULT };

    planning::MotionCommand motion_command;

    /// Set of obstacles added by plays
    rj_geometry::ShapeSet local_obstacles;

    ShootMode shoot_mode = ShootMode::KICK;
    TriggerMode trigger_mode = TriggerMode::STAND_DOWN;
    DribblerMode dribbler_mode = DribblerMode::DEFAULT;
    float kick_speed = 0;

    bool is_active = false;

    int8_t priority = 0;
};

/*
 * @brief overload equality operators to allow RobotIntent==RobotIntent
 */
bool operator==(const RobotIntent& r1, const RobotIntent& r2);
bool operator!=(const RobotIntent& r1, const RobotIntent& r2);

namespace rclcpp {

template <>
struct TypeAdapter<RobotIntent, rj_msgs::msg::RobotIntent> {
    using is_specialized = std::true_type;
    using custom_type = RobotIntent;
    using ros_message_type = rj_msgs::msg::RobotIntent;

    static void convert_to_ros_message(const custom_type& source, ros_message_type& destination) {
        destination.robot_id = static_cast<uint8_t>(source.robot_id);
        rclcpp::TypeAdapter<planning::MotionCommand, rj_msgs::msg::MotionCommand>::
            convert_to_ros_message(source.motion_command, destination.motion_command);
        rclcpp::TypeAdapter<rj_geometry::ShapeSet, rj_geometry_msgs::msg::ShapeSet>::
            convert_to_ros_message(source.local_obstacles, destination.local_obstacles);
        destination.shoot_mode = static_cast<uint8_t>(source.shoot_mode);
        destination.trigger_mode = static_cast<uint8_t>(source.trigger_mode);
        destination.dribbler_mode = static_cast<uint8_t>(source.dribbler_mode);
        destination.kick_speed = source.kick_speed;
        destination.is_active = source.is_active;
        destination.priority = source.priority;
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination.robot_id = static_cast<uint8_t>(source.robot_id);
        rclcpp::TypeAdapter<planning::MotionCommand, rj_msgs::msg::MotionCommand>::convert_to_custom(
            source.motion_command, destination.motion_command);
        rclcpp::TypeAdapter<rj_geometry::ShapeSet, rj_geometry_msgs::msg::ShapeSet>::convert_to_custom(
            source.local_obstacles, destination.local_obstacles);
        destination.shoot_mode = static_cast<RobotIntent::ShootMode>(source.shoot_mode);
        destination.trigger_mode = static_cast<RobotIntent::TriggerMode>(source.trigger_mode);
        destination.dribbler_mode = static_cast<RobotIntent::DribblerMode>(source.dribbler_mode);
        destination.kick_speed = source.kick_speed;
        destination.is_active = source.is_active;
        destination.priority = source.priority;
    }
};

}  // namespace rclcpp
