#pragma once

#include <variant>
#include <vector>

#include <spdlog/spdlog.h>

#include <rj_convert/ros_convert.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry/pose.hpp>
#include <rj_msgs/msg/motion_command.hpp>

#include "rj_common/planning/instant.hpp"
#include "rj_common/planning/trajectory.hpp"
#include "rj_common/world_state.hpp"

namespace planning {

/*
 * Make robot face along its path (for PTMC).
 */
struct FaceTarget {};
bool operator==([[maybe_unused]] const FaceTarget& a, [[maybe_unused]] const FaceTarget& b);

/*
 * Make robot face a specific heading while traveling (for PTMC).
 *
 * TODO(?): heading based on what coord frame? global frame or robot-centric?
 */
struct FaceAngle {
    double target;
};
bool operator==(const FaceAngle& a, const FaceAngle& b);

/*
 * Make robot face a specific point while traveling (for PTMC).
 */
struct FacePoint {
    rj_geometry::Point face_point;
};
bool operator==(const FacePoint& a, const FacePoint& b);

/*
 * Make robot face ball while traveling (for PTMC).
 */
struct FaceBall {};
bool operator==([[maybe_unused]] const FaceBall& a, [[maybe_unused]] const FaceBall& b);

using PathTargetFaceOption = std::variant<FaceTarget, FaceAngle, FacePoint, FaceBall>;

/**
 * TODO(Kevin): fix this doc
 */
struct MotionCommand {
    std::string name{"halt"};
    LinearMotionInstant target{};
    PathTargetFaceOption face_option = FaceTarget{};
    bool ignore_ball{false};
    rj_geometry::Point pivot_point{};
    double pivot_radius{kRobotRadius};
};
bool operator==(const MotionCommand& a, const MotionCommand& b);

}  // namespace planning

namespace rclcpp {

/*
 * These methods allow conversion of ROS .msg types to standard C++ structs.
 * The reason we do this is because though ROS auto-generates C++ structs for
 * each msg (that's why we can use them in code), each auto-generated struct
 * cannot be modified.
 *
 * This is an issue when we say, want to compare msgs with floating point
 * numbers in them: default floating point == comparison is notoriously lousy,
 * but we can't override the ROS structs to implement our own fuzzy floaing
 * point comparison.
 *
 * (This comment should probably be in documentation, but I leave it here for
 * now -Kevin)
 */
template <>
struct TypeAdapter<planning::MotionCommand, rj_msgs::msg::MotionCommand> {
    using is_specialized = std::true_type;
    using custom_type = planning::MotionCommand;
    using ros_message_type = rj_msgs::msg::MotionCommand;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination = ros_message_type{};
        // take the name from the struct and put it in the ROS msg version
        destination.name = source.name;

        // TODO(Kevin): what if these are empty?
        // convert the LinearMotionInstant target
        destination.target.push_back(
            rj_convert::convert_to_ros<planning::LinearMotionInstant,
                                       rj_msgs::msg::LinearMotionInstant>(source.target));

        // convert the PathTargetFaceOptions to the angle override options
        const auto* maybe_point = std::get_if<planning::FacePoint>(&source.face_option);
        const auto* maybe_angle = std::get_if<planning::FaceAngle>(&source.face_option);
        if (maybe_point != nullptr) {
            rj_geometry_msgs::msg::Point face_point =
                rj_convert::convert_to_ros<rj_geometry::Point, rj_geometry_msgs::msg::Point>(
                    maybe_point->face_point);
            destination.override_face_point.push_back(face_point);
        } else if (maybe_angle != nullptr) {
            double face_angle = maybe_angle->target;
            destination.override_angle.push_back(face_angle);
        } else if (std::holds_alternative<planning::FaceBall>(source.face_option)) {
            destination.face_ball.push_back(true);
        }

        // convert the ignore_ball bool
        destination.ignore_ball.push_back(source.ignore_ball);

        // convert pivot point
        destination.pivot_point.push_back(
            rj_convert::convert_to_ros<rj_geometry::Point, rj_geometry_msgs::msg::Point>(
                source.pivot_point));

        // convert the pivot radius
        destination.pivot_radius.push_back(source.pivot_radius);
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        planning::MotionCommand result;

        // take the name from the ROS msg and put it in the struct version
        result.name = source.name;

        // convert the LinearMotionInstant msg to cpp
        if (!source.target.empty()) {
            result.target = rj_convert::convert_from_ros<rj_msgs::msg::LinearMotionInstant,
                                                        planning::LinearMotionInstant>(
                source.target[0]);
        }

        // convert one of the angle overrides to PathTargetFaceOption
        if (!source.override_angle.empty()) {
            result.face_option = planning::FaceAngle{source.override_angle.front()};
        } else if (!source.override_face_point.empty()) {
            result.face_option =
                planning::FacePoint{rj_convert::convert_from_ros<
                    rj_geometry_msgs::msg::Point, rj_geometry::Point>(
                    source.override_face_point.front())};
        } else if (!source.face_ball.empty()) {
            result.face_option = planning::FaceBall{};
        } else {
            // default to facing destination if no other FaceOption given
            result.face_option = planning::FaceTarget{};
        }

        // convert ignore_ball bool
        if (!source.ignore_ball.empty()) {
            result.ignore_ball = source.ignore_ball[0];
        }

        // convert pivot_point
        if (!source.pivot_point.empty()) {
            result.pivot_point = rj_convert::convert_from_ros<
                rj_geometry_msgs::msg::Point, rj_geometry::Point>(source.pivot_point[0]);
        }

        if (!source.pivot_radius.empty()) {
            result.pivot_radius = source.pivot_radius[0];
        }

        destination = result;
    }
};


}  // namespace rclcpp
