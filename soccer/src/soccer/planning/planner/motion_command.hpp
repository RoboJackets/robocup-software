#pragma once

#include <variant>
#include <vector>

#include <rj_convert/ros_convert.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry/pose.hpp>
#include <rj_msgs/msg/collect_motion_command.hpp>
#include <rj_msgs/msg/empty_motion_command.hpp>
#include <rj_msgs/msg/intercept_motion_command.hpp>
#include <rj_msgs/msg/line_kick_motion_command.hpp>
#include <rj_msgs/msg/linear_motion_instant.hpp>
#include <rj_msgs/msg/motion_command.hpp>
#include <rj_msgs/msg/path_target_motion_command.hpp>
#include <rj_msgs/msg/pivot_motion_command.hpp>
#include <rj_msgs/msg/settle_motion_command.hpp>
#include <rj_msgs/msg/world_vel_motion_command.hpp>
#include <world_state.hpp>

#include "planning/instant.hpp"
#include "planning/trajectory.hpp"

namespace Planning {
struct SettleCommand {
    std::optional<rj_geometry::Point> target;
};
struct CollectCommand {};
struct LineKickCommand {
    rj_geometry::Point target;
};

/**
 * An empty "do-nothing" motion command.
 */
struct EmptyCommand {};

struct TargetFaceTangent {};
struct TargetFaceAngle {
    double target;
};
struct TargetFacePoint {
    rj_geometry::Point face_point;
};

using AngleOverride =
    std::variant<TargetFaceTangent, TargetFaceAngle, TargetFacePoint>;
/**
 * Move to a particular target with a particular velocity, avoiding obstacles.
 */
struct PathTargetCommand {
    LinearMotionInstant goal;
    AngleOverride angle_override = TargetFaceTangent{};
};

/**
 * Move with a particular velocity.
 */
struct WorldVelCommand {
    rj_geometry::Point world_vel;
};

/**
 * Pivot around a given point, with a given target angle.
 *
 * The robot will face the pivot_point throughout the command.
 */
struct PivotCommand {
    rj_geometry::Point pivot_point;
    rj_geometry::Point pivot_target;
};

/**
 * Intercept a moving ball, disregarding whether or not we can actually capture
 * it.
 *
 * Designed for goal defense.
 */
struct InterceptCommand {
    rj_geometry::Point target;
};

using MotionCommand =
    std::variant<EmptyCommand, PathTargetCommand, WorldVelCommand, PivotCommand,
                 SettleCommand, CollectCommand, LineKickCommand,
                 InterceptCommand>;

}  // namespace Planning

namespace rj_convert {

template <>
struct RosConverter<Planning::EmptyCommand, rj_msgs::msg::EmptyMotionCommand> {
    static rj_msgs::msg::EmptyMotionCommand to_ros([
        [maybe_unused]] const Planning::EmptyCommand& from) {
        return rj_msgs::msg::EmptyMotionCommand{};
    }

    static Planning::EmptyCommand from_ros([
        [maybe_unused]] const rj_msgs::msg::EmptyMotionCommand& from) {
        return Planning::EmptyCommand{};
    }
};

ASSOCIATE_CPP_ROS(Planning::EmptyCommand, rj_msgs::msg::EmptyMotionCommand);

template <>
struct RosConverter<Planning::PathTargetCommand, rj_msgs::msg::PathTargetMotionCommand> {
    static rj_msgs::msg::PathTargetMotionCommand to_ros([
        [maybe_unused]] const Planning::PathTargetCommand& from) {
        return rj_msgs::build<rj_msgs::msg::PathTargetMotionCommand>().target(
            convert_to_ros(from.goal));
    }

    static Planning::PathTargetCommand from_ros([
        [maybe_unused]] const rj_msgs::msg::PathTargetMotionCommand& from) {
        return Planning::PathTargetCommand{convert_from_ros(from.target)};
    }
};

ASSOCIATE_CPP_ROS(Planning::PathTargetCommand, rj_msgs::msg::PathTargetMotionCommand);

template <>
struct RosConverter<Planning::WorldVelCommand, rj_msgs::msg::WorldVelMotionCommand> {
    static rj_msgs::msg::WorldVelMotionCommand to_ros([
        [maybe_unused]] const Planning::WorldVelCommand& from) {
        return rj_msgs::build<rj_msgs::msg::WorldVelMotionCommand>().world_vel(
            convert_to_ros(from.world_vel));
    }

    static Planning::WorldVelCommand from_ros([
        [maybe_unused]] const rj_msgs::msg::WorldVelMotionCommand& from) {
        return Planning::WorldVelCommand{convert_from_ros(from.world_vel)};
    }
};

ASSOCIATE_CPP_ROS(Planning::WorldVelCommand, rj_msgs::msg::WorldVelMotionCommand);

template <>
struct RosConverter<Planning::PivotCommand, rj_msgs::msg::PivotMotionCommand> {
    static rj_msgs::msg::PivotMotionCommand to_ros([
        [maybe_unused]] const Planning::PivotCommand& from) {
        return rj_msgs::build<rj_msgs::msg::PivotMotionCommand>()
            .pivot_point(convert_to_ros(from.pivot_point))
            .pivot_target(convert_to_ros(from.pivot_target));
    }

    static Planning::PivotCommand from_ros([
        [maybe_unused]] const rj_msgs::msg::PivotMotionCommand& from) {
        return Planning::PivotCommand{convert_from_ros(from.pivot_point),
                                      convert_from_ros(from.pivot_target)};
    }
};

ASSOCIATE_CPP_ROS(Planning::PivotCommand, rj_msgs::msg::PivotMotionCommand);

template <>
struct RosConverter<Planning::SettleCommand, rj_msgs::msg::SettleMotionCommand> {
    static rj_msgs::msg::SettleMotionCommand to_ros([
        [maybe_unused]] const Planning::SettleCommand& from) {
        rj_msgs::msg::SettleMotionCommand result;
        if (from.target.has_value()) {
            result.maybe_target.push_back(convert_to_ros(from.target.value()));
        }
        return result;
    }

    static Planning::SettleCommand from_ros([
        [maybe_unused]] const rj_msgs::msg::SettleMotionCommand& from) {
        Planning::SettleCommand result;
        if (!from.maybe_target.empty()) {
            result.target = convert_from_ros(from.maybe_target.front());
        }
        return result;
    }
};

ASSOCIATE_CPP_ROS(Planning::SettleCommand, rj_msgs::msg::SettleMotionCommand);

template <>
struct RosConverter<Planning::CollectCommand, rj_msgs::msg::CollectMotionCommand> {
    static rj_msgs::msg::CollectMotionCommand to_ros([
        [maybe_unused]] const Planning::CollectCommand& from) {
        return rj_msgs::build<rj_msgs::msg::CollectMotionCommand>();
    }

    static Planning::CollectCommand from_ros([
        [maybe_unused]] const rj_msgs::msg::CollectMotionCommand& from) {
        return Planning::CollectCommand{};
    }
};

ASSOCIATE_CPP_ROS(Planning::CollectCommand, rj_msgs::msg::CollectMotionCommand);

template <>
struct RosConverter<Planning::LineKickCommand, rj_msgs::msg::LineKickMotionCommand> {
    static rj_msgs::msg::LineKickMotionCommand to_ros([
        [maybe_unused]] const Planning::LineKickCommand& from) {
        return rj_msgs::build<rj_msgs::msg::LineKickMotionCommand>().target(
            convert_to_ros(from.target));
    }

    static Planning::LineKickCommand from_ros([
        [maybe_unused]] const rj_msgs::msg::LineKickMotionCommand& from) {
        return Planning::LineKickCommand{convert_from_ros(from.target)};
    }
};

ASSOCIATE_CPP_ROS(Planning::LineKickCommand, rj_msgs::msg::LineKickMotionCommand);

template <>
struct RosConverter<Planning::InterceptCommand, rj_msgs::msg::InterceptMotionCommand> {
    static rj_msgs::msg::InterceptMotionCommand to_ros([
        [maybe_unused]] const Planning::InterceptCommand& from) {
        return rj_msgs::build<rj_msgs::msg::InterceptMotionCommand>().target(
            convert_to_ros(from.target));
    }

    static Planning::InterceptCommand from_ros([
        [maybe_unused]] const rj_msgs::msg::InterceptMotionCommand& from) {
        return Planning::InterceptCommand{convert_from_ros(from.target)};
    }
};

ASSOCIATE_CPP_ROS(Planning::InterceptCommand, rj_msgs::msg::InterceptMotionCommand);

template <>
struct RosConverter<Planning::MotionCommand, rj_msgs::msg::MotionCommand> {
    static rj_msgs::msg::MotionCommand to_ros(const Planning::MotionCommand& from) {
        rj_msgs::msg::MotionCommand result;
        if (const auto* empty = std::get_if<Planning::EmptyCommand>(&from)) {
            result.empty_command.emplace_back(convert_to_ros(*empty));
        } else if (const auto* path_target = std::get_if<Planning::PathTargetCommand>(&from)) {
            result.path_target_command.emplace_back(convert_to_ros(*path_target));
        } else if (const auto* world_vel = std::get_if<Planning::WorldVelCommand>(&from)) {
            result.world_vel_command.emplace_back(convert_to_ros(*world_vel));
        } else if (const auto* pivot = std::get_if<Planning::PivotCommand>(&from)) {
            result.pivot_command.emplace_back(convert_to_ros(*pivot));
        } else if (const auto* settle = std::get_if<Planning::SettleCommand>(&from)) {
            result.settle_command.emplace_back(convert_to_ros(*settle));
        } else if (const auto* collect = std::get_if<Planning::CollectCommand>(&from)) {
            result.collect_command.emplace_back(convert_to_ros(*collect));
        } else if (const auto* line_kick = std::get_if<Planning::LineKickCommand>(&from)) {
            result.line_kick_command.emplace_back(convert_to_ros(*line_kick));
        } else if (const auto* intercept = std::get_if<Planning::InterceptCommand>(&from)) {
            result.intercept_command.emplace_back(convert_to_ros(*intercept));
        } else {
            throw std::runtime_error("Invalid variant of MotionCommand");
        }
        return result;
    }

    static Planning::MotionCommand from_ros(const rj_msgs::msg::MotionCommand& from) {
        Planning::MotionCommand result;
        if (!from.empty_command.empty()) {
            result = convert_from_ros(from.empty_command.front());
        } else if (!from.path_target_command.empty()) {
            result = convert_from_ros(from.path_target_command.front());
        } else if (!from.world_vel_command.empty()) {
            result = convert_from_ros(from.world_vel_command.front());
        } else if (!from.pivot_command.empty()) {
            result = convert_from_ros(from.pivot_command.front());
        } else if (!from.settle_command.empty()) {
            result = convert_from_ros(from.settle_command.front());
        } else if (!from.collect_command.empty()) {
            result = convert_from_ros(from.collect_command.front());
        } else if (!from.line_kick_command.empty()) {
            result = convert_from_ros(from.line_kick_command.front());
        } else if (!from.intercept_command.empty()) {
            result = convert_from_ros(from.intercept_command.front());
        } else {
            throw std::runtime_error("Invalid variant of MotionCommand");
        }
        return result;
    }
};

ASSOCIATE_CPP_ROS(Planning::MotionCommand, rj_msgs::msg::MotionCommand);

}  // namespace rj_convert
