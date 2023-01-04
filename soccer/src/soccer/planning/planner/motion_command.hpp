#pragma once

#include <variant>
#include <vector>

#include <spdlog/spdlog.h>

#include <rj_convert/ros_convert.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry/pose.hpp>
#include <rj_msgs/msg/collect_motion_command.hpp>
#include <rj_msgs/msg/empty_motion_command.hpp>
#include <rj_msgs/msg/goalie_idle_motion_command.hpp>
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

namespace planning {
struct SettleMotionCommand {
    std::optional<rj_geometry::Point> target;
};
bool operator==(const SettleMotionCommand& a, const SettleMotionCommand& b);

struct CollectMotionCommand {};
bool operator==([[maybe_unused]] const CollectMotionCommand& a,
                [[maybe_unused]] const CollectMotionCommand& b);

struct LineKickMotionCommand {
    rj_geometry::Point target;
};
bool operator==(const LineKickMotionCommand& a, const LineKickMotionCommand& b);

/**
 * An empty "do-nothing" motion command.
 */
struct EmptyMotionCommand {};
bool operator==([[maybe_unused]] const EmptyMotionCommand& a,
                [[maybe_unused]] const EmptyMotionCommand& b);

/*
 * Make robot face along its path (for PTMC).
 */
struct TargetFaceTangent {};
bool operator==([[maybe_unused]] const TargetFaceTangent& a,
                [[maybe_unused]] const TargetFaceTangent& b);

/*
 * Make robot face a specific heading while traveling (for PTMC).
 *
 * TODO(?): heading based on what coord frame? global frame or robot-centric?
 */
struct TargetFaceAngle {
    double target;
};
bool operator==(const TargetFaceAngle& a, const TargetFaceAngle& b);

/*
 * Make robot face a specific point while traveling (for PTMC).
 */
struct TargetFacePoint {
    rj_geometry::Point face_point;
};
bool operator==(const TargetFacePoint& a, const TargetFacePoint& b);

/*
 * Make robot face ball while traveling (for PTMC).
 */
struct TargetFaceBall {};
bool operator==([[maybe_unused]] const TargetFaceBall& a, [[maybe_unused]] const TargetFaceBall& b);

using AngleOverride =
    std::variant<TargetFaceTangent, TargetFaceAngle, TargetFacePoint, TargetFaceBall>;
/**
 * Move to a particular target with a particular ending velocity, avoiding obstacles.
 */
struct PathTargetMotionCommand {
    LinearMotionInstant goal{};
    AngleOverride angle_override = TargetFaceTangent{};
    bool ignore_ball = false;
};
bool operator==(const PathTargetMotionCommand& a, const PathTargetMotionCommand& b);

/**
 * Move with a particular velocity.
 */
struct WorldVelMotionCommand {
    rj_geometry::Point world_vel;
};
bool operator==(const WorldVelMotionCommand& a, const WorldVelMotionCommand& b);

/**
 * Pivot around a given point, with a given target angle.
 *
 * The robot will face the pivot_point throughout the command.
 */
struct PivotMotionCommand {
    rj_geometry::Point pivot_point;
    rj_geometry::Point pivot_target;
};
bool operator==(const PivotMotionCommand& a, const PivotMotionCommand& b);

/**
 * Intercept a moving ball, disregarding whether or not we can actually capture
 * it.
 *
 * Designed for goal defense.
 */
struct InterceptMotionCommand {
    rj_geometry::Point target;
};
bool operator==(const InterceptMotionCommand& a, const InterceptMotionCommand& b);

/*
 * Make the Goalie track the ball when not saving shots.
 */
struct GoalieIdleMotionCommand {};
bool operator==([[maybe_unused]] const GoalieIdleMotionCommand& a,
                [[maybe_unused]] const GoalieIdleMotionCommand& b);

using MotionCommand =
    std::variant<EmptyMotionCommand, PathTargetMotionCommand, WorldVelMotionCommand,
                 PivotMotionCommand, SettleMotionCommand, CollectMotionCommand,
                 LineKickMotionCommand, InterceptMotionCommand, GoalieIdleMotionCommand>;

}  // namespace planning

namespace rj_convert {

template <>
struct RosConverter<planning::EmptyMotionCommand, rj_msgs::msg::EmptyMotionCommand> {
    static rj_msgs::msg::EmptyMotionCommand to_ros(
        [[maybe_unused]] const planning::EmptyMotionCommand& from) {
        return rj_msgs::msg::EmptyMotionCommand{};
    }

    static planning::EmptyMotionCommand from_ros(
        [[maybe_unused]] const rj_msgs::msg::EmptyMotionCommand& from) {
        return planning::EmptyMotionCommand{};
    }
};

ASSOCIATE_CPP_ROS(planning::EmptyMotionCommand, rj_msgs::msg::EmptyMotionCommand);

template <>
struct RosConverter<planning::PathTargetMotionCommand, rj_msgs::msg::PathTargetMotionCommand> {
    static rj_msgs::msg::PathTargetMotionCommand to_ros(
        const planning::PathTargetMotionCommand& from) {
        rj_msgs::msg::PathTargetMotionCommand result;
        result.target = convert_to_ros(from.goal);

        const auto* maybe_point = std::get_if<planning::TargetFacePoint>(&from.angle_override);
        const auto* maybe_angle = std::get_if<planning::TargetFaceAngle>(&from.angle_override);
        if (maybe_point != nullptr) {
            rj_geometry_msgs::msg::Point face_point = convert_to_ros(maybe_point->face_point);
            result.override_face_point.push_back(face_point);
        } else if (maybe_angle != nullptr) {
            double face_angle = maybe_angle->target;
            result.override_angle.push_back(face_angle);
        } else if (std::holds_alternative<planning::TargetFaceBall>(from.angle_override)) {
            result.face_ball = true;
        }

        result.ignore_ball = from.ignore_ball;

        return result;
    }

    static planning::PathTargetMotionCommand from_ros(
        const rj_msgs::msg::PathTargetMotionCommand& from) {
        planning::PathTargetMotionCommand result;
        result.goal = convert_from_ros(from.target);
        if (!from.override_angle.empty()) {
            result.angle_override = planning::TargetFaceAngle{from.override_angle.front()};
        } else if (!from.override_face_point.empty()) {
            result.angle_override =
                planning::TargetFacePoint{convert_from_ros(from.override_face_point.front())};
        } else if (from.face_ball) {
            result.angle_override = planning::TargetFaceBall{};
        } else {
            // default to facing destination if no other AngleOverride given
            result.angle_override = planning::TargetFaceTangent{};
        }
        result.ignore_ball = from.ignore_ball;
        return result;
    }
};

ASSOCIATE_CPP_ROS(planning::PathTargetMotionCommand, rj_msgs::msg::PathTargetMotionCommand);

template <>
struct RosConverter<planning::WorldVelMotionCommand, rj_msgs::msg::WorldVelMotionCommand> {
    static rj_msgs::msg::WorldVelMotionCommand to_ros(const planning::WorldVelMotionCommand& from) {
        return rj_msgs::build<rj_msgs::msg::WorldVelMotionCommand>().world_vel(
            convert_to_ros(from.world_vel));
    }

    static planning::WorldVelMotionCommand from_ros(
        const rj_msgs::msg::WorldVelMotionCommand& from) {
        return planning::WorldVelMotionCommand{convert_from_ros(from.world_vel)};
    }
};

ASSOCIATE_CPP_ROS(planning::WorldVelMotionCommand, rj_msgs::msg::WorldVelMotionCommand);

template <>
struct RosConverter<planning::PivotMotionCommand, rj_msgs::msg::PivotMotionCommand> {
    static rj_msgs::msg::PivotMotionCommand to_ros(const planning::PivotMotionCommand& from) {
        return rj_msgs::build<rj_msgs::msg::PivotMotionCommand>()
            .pivot_point(convert_to_ros(from.pivot_point))
            .pivot_target(convert_to_ros(from.pivot_target));
    }

    static planning::PivotMotionCommand from_ros(const rj_msgs::msg::PivotMotionCommand& from) {
        return planning::PivotMotionCommand{convert_from_ros(from.pivot_point),
                                            convert_from_ros(from.pivot_target)};
    }
};

ASSOCIATE_CPP_ROS(planning::PivotMotionCommand, rj_msgs::msg::PivotMotionCommand);

template <>
struct RosConverter<planning::SettleMotionCommand, rj_msgs::msg::SettleMotionCommand> {
    static rj_msgs::msg::SettleMotionCommand to_ros(const planning::SettleMotionCommand& from) {
        rj_msgs::msg::SettleMotionCommand result;
        if (from.target.has_value()) {
            result.maybe_target.push_back(convert_to_ros(from.target.value()));
        }
        return result;
    }

    static planning::SettleMotionCommand from_ros(const rj_msgs::msg::SettleMotionCommand& from) {
        planning::SettleMotionCommand result;
        if (!from.maybe_target.empty()) {
            result.target = convert_from_ros(from.maybe_target.front());
        }
        return result;
    }
};

ASSOCIATE_CPP_ROS(planning::SettleMotionCommand, rj_msgs::msg::SettleMotionCommand);

template <>
struct RosConverter<planning::CollectMotionCommand, rj_msgs::msg::CollectMotionCommand> {
    static rj_msgs::msg::CollectMotionCommand to_ros(
        [[maybe_unused]] const planning::CollectMotionCommand& from) {
        return rj_msgs::build<rj_msgs::msg::CollectMotionCommand>();
    }

    static planning::CollectMotionCommand from_ros(
        [[maybe_unused]] const rj_msgs::msg::CollectMotionCommand& from) {
        return planning::CollectMotionCommand{};
    }
};

ASSOCIATE_CPP_ROS(planning::CollectMotionCommand, rj_msgs::msg::CollectMotionCommand);

template <>
struct RosConverter<planning::GoalieIdleMotionCommand, rj_msgs::msg::GoalieIdleMotionCommand> {
    // clang-format is disagreeing with itself here, so I disabled it for this block
    // clang-format off
    static rj_msgs::msg::GoalieIdleMotionCommand to_ros(
        [[maybe_unused]] const planning::GoalieIdleMotionCommand& from) {
        return rj_msgs::build<rj_msgs::msg::GoalieIdleMotionCommand>();
    }

    static planning::GoalieIdleMotionCommand from_ros(
        [[maybe_unused]] const rj_msgs::msg::GoalieIdleMotionCommand& from) {
        return planning::GoalieIdleMotionCommand{};
    }
    // clang-format on
};

ASSOCIATE_CPP_ROS(planning::GoalieIdleMotionCommand, rj_msgs::msg::GoalieIdleMotionCommand);

template <>
struct RosConverter<planning::LineKickMotionCommand, rj_msgs::msg::LineKickMotionCommand> {
    static rj_msgs::msg::LineKickMotionCommand to_ros(
        [[maybe_unused]] const planning::LineKickMotionCommand& from) {
        return rj_msgs::build<rj_msgs::msg::LineKickMotionCommand>().target(
            convert_to_ros(from.target));
    }

    static planning::LineKickMotionCommand from_ros(
        [[maybe_unused]] const rj_msgs::msg::LineKickMotionCommand& from) {
        return planning::LineKickMotionCommand{convert_from_ros(from.target)};
    }
};

ASSOCIATE_CPP_ROS(planning::LineKickMotionCommand, rj_msgs::msg::LineKickMotionCommand);

template <>
struct RosConverter<planning::InterceptMotionCommand, rj_msgs::msg::InterceptMotionCommand> {
    static rj_msgs::msg::InterceptMotionCommand to_ros(
        [[maybe_unused]] const planning::InterceptMotionCommand& from) {
        return rj_msgs::build<rj_msgs::msg::InterceptMotionCommand>().target(
            convert_to_ros(from.target));
    }

    static planning::InterceptMotionCommand from_ros(
        [[maybe_unused]] const rj_msgs::msg::InterceptMotionCommand& from) {
        return planning::InterceptMotionCommand{convert_from_ros(from.target)};
    }
};

ASSOCIATE_CPP_ROS(planning::InterceptMotionCommand, rj_msgs::msg::InterceptMotionCommand);

template <>
struct RosConverter<planning::MotionCommand, rj_msgs::msg::MotionCommand> {
    static rj_msgs::msg::MotionCommand to_ros(const planning::MotionCommand& from) {
        // TODO(Kevin): wtf is this
        rj_msgs::msg::MotionCommand result;
        if (const auto* empty = std::get_if<planning::EmptyMotionCommand>(&from)) {
            result.empty_command.emplace_back(convert_to_ros(*empty));
        } else if (const auto* path_target =
                       std::get_if<planning::PathTargetMotionCommand>(&from)) {
            result.path_target_command.emplace_back(convert_to_ros(*path_target));
        } else if (const auto* world_vel = std::get_if<planning::WorldVelMotionCommand>(&from)) {
            result.world_vel_command.emplace_back(convert_to_ros(*world_vel));
        } else if (const auto* pivot = std::get_if<planning::PivotMotionCommand>(&from)) {
            result.pivot_command.emplace_back(convert_to_ros(*pivot));
        } else if (const auto* settle = std::get_if<planning::SettleMotionCommand>(&from)) {
            result.settle_command.emplace_back(convert_to_ros(*settle));
        } else if (const auto* collect = std::get_if<planning::CollectMotionCommand>(&from)) {
            result.collect_command.emplace_back(convert_to_ros(*collect));
        } else if (const auto* line_kick = std::get_if<planning::LineKickMotionCommand>(&from)) {
            result.line_kick_command.emplace_back(convert_to_ros(*line_kick));
        } else if (const auto* intercept = std::get_if<planning::InterceptMotionCommand>(&from)) {
            result.intercept_command.emplace_back(convert_to_ros(*intercept));
        } else if (const auto* goalie_idle =
                       std::get_if<planning::GoalieIdleMotionCommand>(&from)) {
            result.goalie_idle_command.emplace_back(convert_to_ros(*goalie_idle));
        } else {
            throw std::runtime_error("Invalid variant of MotionCommand");
        }
        return result;
    }

    static planning::MotionCommand from_ros(const rj_msgs::msg::MotionCommand& from) {
        planning::MotionCommand result;
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
        } else if (!from.goalie_idle_command.empty()) {
            result = convert_from_ros(from.goalie_idle_command.front());
        } else {
            throw std::runtime_error("Invalid variant of MotionCommand");
        }
        return result;
    }
};

ASSOCIATE_CPP_ROS(planning::MotionCommand, rj_msgs::msg::MotionCommand);

}  // namespace rj_convert
