#pragma once

#include <variant>
#include <vector>

#include <spdlog/spdlog.h>

#include <rj_convert/ros_convert.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry/pose.hpp>
#include <rj_msgs/msg/motion_command.hpp>
#include <world_state.hpp>

#include "planning/instant.hpp"
#include "planning/trajectory.hpp"

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
    double waller_radius{0};
    uint8_t waller_id{};
    uint8_t num_wallers{};
    uint8_t waller_parent{};
};
bool operator==(const MotionCommand& a, const MotionCommand& b);

}  // namespace planning

namespace rj_convert {

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
struct RosConverter<planning::MotionCommand, rj_msgs::msg::MotionCommand> {
    static rj_msgs::msg::MotionCommand to_ros(const planning::MotionCommand& from) {
        rj_msgs::msg::MotionCommand result;

        // take the name from the struct and put it in the ROS msg version
        result.name = from.name;

        // TODO(Kevin): what if these are empty?
        // convert the LinearMotionInstant target
        result.target.push_back(convert_to_ros(from.target));

        // convert the PathTargetFaceOptions to the angle override options
        const auto* maybe_point = std::get_if<planning::FacePoint>(&from.face_option);
        const auto* maybe_angle = std::get_if<planning::FaceAngle>(&from.face_option);
        if (maybe_point != nullptr) {
            rj_geometry_msgs::msg::Point face_point = convert_to_ros(maybe_point->face_point);
            result.override_face_point.push_back(face_point);
        } else if (maybe_angle != nullptr) {
            double face_angle = maybe_angle->target;
            result.override_angle.push_back(face_angle);
        } else if (std::holds_alternative<planning::FaceBall>(from.face_option)) {
            result.face_ball.push_back(true);
        }

        // convert the ignore_ball bool
        result.ignore_ball.push_back(from.ignore_ball);

        // convert pivot point
        result.pivot_point.push_back(convert_to_ros(from.pivot_point));

        // convert the pivot radius
        result.pivot_radius.push_back(from.pivot_radius);

        // convert pivot point
        result.waller_parent.push_back(from.waller_parent);

        // convert the pivot radius
        result.waller_radius.push_back(from.pivot_radius);     

        result.waller_id.push_back(from.waller_id);   
        
        result.num_wallers.push_back(from.num_wallers);   

        return result;
    }

    static planning::MotionCommand from_ros(const rj_msgs::msg::MotionCommand& from) {
        planning::MotionCommand result;

        // take the name from the ROS msg and put it in the struct version
        result.name = from.name;

        // convert the LinearMotionInstant msg to cpp
        if (!from.target.empty()) {
            result.target = convert_from_ros(from.target[0]);
        }

        // convert one of the angle overrides to PathTargetFaceOption
        if (!from.override_angle.empty()) {
            result.face_option = planning::FaceAngle{from.override_angle.front()};
        } else if (!from.override_face_point.empty()) {
            result.face_option =
                planning::FacePoint{convert_from_ros(from.override_face_point.front())};
        } else if (!from.face_ball.empty()) {
            result.face_option = planning::FaceBall{};
        } else {
            // default to facing destination if no other FaceOption given
            result.face_option = planning::FaceTarget{};
        }

        // convert ignore_ball bool
        if (!from.ignore_ball.empty()) {
            result.ignore_ball = from.ignore_ball[0];
        }

        // convert pivot_point
        if (!from.pivot_point.empty()) {
            result.pivot_point = convert_from_ros(from.pivot_point[0]);
        }

        if (!from.pivot_radius.empty()) {
            result.pivot_radius = from.pivot_radius[0];
        }

        // convert waller
        if (!from.waller_parent.empty()) {
            result.waller_parent = from.waller_parent[0];
        }

        if (!from.waller_radius.empty()) {
            result.waller_radius = from.waller_radius[0];
        }

        if (!from.waller_id.empty()) {
            result.waller_id = from.waller_id[0];
        }


        if (!from.num_wallers.empty()) {
            result.num_wallers = from.num_wallers[0];
        }

        return result;
    }
};

ASSOCIATE_CPP_ROS(planning::MotionCommand, rj_msgs::msg::MotionCommand);

}  // namespace rj_convert
