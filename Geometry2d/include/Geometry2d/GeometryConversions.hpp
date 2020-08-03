#pragma once

#include <Geometry2d/Line.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/Pose.hpp>
#include <Geometry2d/Rect.hpp>
#include <Geometry2d/Segment.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_geometry_msgs/msg/line.hpp>
#include <rj_geometry_msgs/msg/point.hpp>
#include <rj_geometry_msgs/msg/pose.hpp>
#include <rj_geometry_msgs/msg/rect.hpp>
#include <rj_geometry_msgs/msg/segment.hpp>
#include <rj_geometry_msgs/msg/twist.hpp>

namespace rj_convert {

template <>
struct RosConverter<Geometry2d::Point, rj_geometry_msgs::msg::Point> {
    static rj_geometry_msgs::msg::Point to_ros(const Geometry2d::Point& from) {
        rj_geometry_msgs::msg::Point to;
        convert_to_ros(from.x(), &to.x);
        convert_to_ros(from.y(), &to.y);
        return to;
    }

    static Geometry2d::Point from_ros(
        const rj_geometry_msgs::msg::Point& from) {
        Geometry2d::Point to;
        convert_from_ros(from.x, &to.x());
        convert_from_ros(from.y, &to.y());
        return to;
    }
};

template <>
struct RosConverter<Geometry2d::Pose, rj_geometry_msgs::msg::Pose> {
    static rj_geometry_msgs::msg::Pose to_ros(const Geometry2d::Pose& from) {
        rj_geometry_msgs::msg::Pose to;
        convert_to_ros(from.position(), &to.position);
        convert_to_ros(from.heading(), &to.heading);
        return to;
    }

    static Geometry2d::Pose from_ros(const rj_geometry_msgs::msg::Pose& from) {
        Geometry2d::Pose to;
        convert_from_ros(from.position, &to.position());
        convert_from_ros(from.heading, &to.heading());
        return to;
    }
};

template <>
struct RosConverter<Geometry2d::Twist, rj_geometry_msgs::msg::Twist> {
    static rj_geometry_msgs::msg::Twist to_ros(const Geometry2d::Twist& from) {
        rj_geometry_msgs::msg::Twist to;
        convert_to_ros(from.linear(), &to.linear);
        convert_to_ros(from.angular(), &to.angular);
        return to;
    }

    static Geometry2d::Twist from_ros(
        const rj_geometry_msgs::msg::Twist& from) {
        Geometry2d::Twist to;
        convert_from_ros(from.linear, &to.linear());
        convert_from_ros(from.angular, &to.angular());
        return to;
    }
};

template <>
struct RosConverter<Geometry2d::Line, rj_geometry_msgs::msg::Line> {
    static rj_geometry_msgs::msg::Line to_ros(const Geometry2d::Line& from) {
        rj_geometry_msgs::msg::Line to;
        convert_to_ros(from.pt, &to.pt);
        return to;
    }

    static Geometry2d::Line from_ros(const rj_geometry_msgs::msg::Line& from) {
        Geometry2d::Line to;
        convert_from_ros(from.pt, &to.pt);
        return to;
    }
};

template <>
struct RosConverter<Geometry2d::Segment, rj_geometry_msgs::msg::Segment> {
    static rj_geometry_msgs::msg::Segment to_ros(
        const Geometry2d::Segment& from) {
        rj_geometry_msgs::msg::Segment to;
        convert_to_ros(from.pt, &to.pt);
        return to;
    }

    static Geometry2d::Segment from_ros(
        const rj_geometry_msgs::msg::Segment& from) {
        Geometry2d::Segment to;
        convert_from_ros(from.pt, &to.pt);
        return to;
    }
};

template <>
struct RosConverter<Geometry2d::Rect, rj_geometry_msgs::msg::Rect> {
    static rj_geometry_msgs::msg::Rect to_ros(const Geometry2d::Rect& from) {
        rj_geometry_msgs::msg::Rect to;
        convert_to_ros(from.pt, &to.pt);
        return to;
    }

    static Geometry2d::Rect from_ros(const rj_geometry_msgs::msg::Rect& from) {
        Geometry2d::Rect to;
        convert_from_ros(from.pt, &to.pt);
        return to;
    }
};

}  // namespace rj_convert