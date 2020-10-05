#pragma once

#include <rj_constants/constants.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_geometry/shape_set.hpp>
#include <rj_msgs/msg/robot_intent.hpp>

#include "planning/planner/motion_command.hpp"
#include "planning/rotation_command.hpp"

struct RobotIntent {
    using Msg = rj_msgs::msg::RobotIntent;
    enum class ShootMode { KICK, CHIP };
    enum class TriggerMode { STAND_DOWN, IMMEDIATE, ON_BREAK_BEAM };

    Planning::MotionCommand motion_command;

    /// Set of obstacles added by plays
    rj_geometry::ShapeSet local_obstacles;

    ShootMode shoot_mode = ShootMode::KICK;
    TriggerMode trigger_mode = TriggerMode::STAND_DOWN;
    float kick_speed = 0;
    float dribbler_speed = 0;

    bool is_active = false;

    int8_t priority = 0;
};

namespace rj_convert {

template <>
struct RosConverter<RobotIntent, rj_msgs::msg::RobotIntent> {
    static rj_msgs::msg::RobotIntent to_ros(const RobotIntent& from) {
        return rj_msgs::build<rj_msgs::msg::RobotIntent>()
            .motion_command(convert_to_ros(from.motion_command))
            .local_obstacles(convert_to_ros(from.local_obstacles))
            .shoot_mode(static_cast<uint8_t>(from.shoot_mode))
            .trigger_mode(static_cast<uint8_t>(from.trigger_mode))
            .kick_speed(convert_to_ros(from.kick_speed))
            .dribbler_speed(convert_to_ros(from.dribbler_speed))
            .is_active(convert_to_ros(from.is_active))
            .priority(convert_to_ros(from.priority));
    }

    static RobotIntent from_ros(const rj_msgs::msg::RobotIntent& from) {
        RobotIntent result;
        result.motion_command = convert_from_ros(from.motion_command);
        result.local_obstacles = convert_from_ros(from.local_obstacles);
        result.shoot_mode = static_cast<RobotIntent::ShootMode>(from.shoot_mode);
        result.trigger_mode = static_cast<RobotIntent::TriggerMode>(from.trigger_mode);
        result.kick_speed = convert_from_ros(from.kick_speed);
        result.dribbler_speed = convert_from_ros(from.dribbler_speed);
        result.is_active = convert_from_ros(from.is_active);
        result.priority = convert_from_ros(from.priority);
        return result;
    }
};

ASSOCIATE_CPP_ROS(RobotIntent, rj_msgs::msg::RobotIntent);

}  // namespace rj_convert