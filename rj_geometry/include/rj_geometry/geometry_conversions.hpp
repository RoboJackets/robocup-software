#pragma once

#include <algorithm>

#include <rj_convert/ros_convert.hpp>
#include <rj_geometry/line.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry/pose.hpp>
#include <rj_geometry/polygon.hpp>
#include <rj_geometry/rect.hpp>
#include <rj_geometry/segment.hpp>
#include <rj_geometry/shape_set.hpp>
#include <rj_geometry_msgs/msg/line.hpp>
#include <rj_geometry_msgs/msg/point.hpp>
#include <rj_geometry_msgs/msg/pose.hpp>
#include <rj_geometry_msgs/msg/polygon.hpp>
#include <rj_geometry_msgs/msg/rect.hpp>
#include <rj_geometry_msgs/msg/segment.hpp>
#include <rj_geometry_msgs/msg/shape_set.hpp>
#include <rj_geometry_msgs/msg/twist.hpp>

namespace rj_convert {

template <>
struct RosConverter<rj_geometry::Point, rj_geometry_msgs::msg::Point> {
    static rj_geometry_msgs::msg::Point to_ros(const rj_geometry::Point& from) {
        rj_geometry_msgs::msg::Point to;
        convert_to_ros(from.x(), &to.x);
        convert_to_ros(from.y(), &to.y);
        return to;
    }

    static rj_geometry::Point from_ros(
        const rj_geometry_msgs::msg::Point& from) {
        rj_geometry::Point to;
        convert_from_ros(from.x, &to.x());
        convert_from_ros(from.y, &to.y());
        return to;
    }
};

ASSOCIATE_CPP_ROS(rj_geometry::Point, rj_geometry::Point::Msg);

template <>
struct RosConverter<rj_geometry::Pose, rj_geometry_msgs::msg::Pose> {
    static rj_geometry_msgs::msg::Pose to_ros(const rj_geometry::Pose& from) {
        rj_geometry_msgs::msg::Pose to;
        convert_to_ros(from.position(), &to.position);
        convert_to_ros(from.heading(), &to.heading);
        return to;
    }

    static rj_geometry::Pose from_ros(const rj_geometry_msgs::msg::Pose& from) {
        rj_geometry::Pose to;
        convert_from_ros(from.position, &to.position());
        convert_from_ros(from.heading, &to.heading());
        return to;
    }
};

ASSOCIATE_CPP_ROS(rj_geometry::Pose, rj_geometry::Pose::Msg);

template <>
struct RosConverter<rj_geometry::Twist, rj_geometry_msgs::msg::Twist> {
    static rj_geometry_msgs::msg::Twist to_ros(const rj_geometry::Twist& from) {
        rj_geometry_msgs::msg::Twist to;
        convert_to_ros(from.linear(), &to.linear);
        convert_to_ros(from.angular(), &to.angular);
        return to;
    }

    static rj_geometry::Twist from_ros(
        const rj_geometry_msgs::msg::Twist& from) {
        rj_geometry::Twist to;
        convert_from_ros(from.linear, &to.linear());
        convert_from_ros(from.angular, &to.angular());
        return to;
    }
};

ASSOCIATE_CPP_ROS(rj_geometry::Twist, rj_geometry::Twist::Msg);

template <>
struct RosConverter<rj_geometry::Line, rj_geometry_msgs::msg::Line> {
    static rj_geometry_msgs::msg::Line to_ros(const rj_geometry::Line& from) {
        rj_geometry_msgs::msg::Line to;
        convert_to_ros(from.pt, &to.pt);
        return to;
    }

    static rj_geometry::Line from_ros(const rj_geometry_msgs::msg::Line& from) {
        rj_geometry::Line to;
        convert_from_ros(from.pt, &to.pt);
        return to;
    }
};

ASSOCIATE_CPP_ROS(rj_geometry::Line, rj_geometry::Line::Msg);

template <>
struct RosConverter<rj_geometry::Segment, rj_geometry_msgs::msg::Segment> {
    static rj_geometry_msgs::msg::Segment to_ros(
        const rj_geometry::Segment& from) {
        rj_geometry_msgs::msg::Segment to;
        convert_to_ros(from.pt, &to.pt);
        return to;
    }

    static rj_geometry::Segment from_ros(
        const rj_geometry_msgs::msg::Segment& from) {
        rj_geometry::Segment to;
        convert_from_ros(from.pt, &to.pt);
        return to;
    }
};

ASSOCIATE_CPP_ROS(rj_geometry::Segment, rj_geometry::Segment::Msg);

template <>
struct RosConverter<rj_geometry::Rect, rj_geometry_msgs::msg::Rect> {
    static rj_geometry_msgs::msg::Rect to_ros(const rj_geometry::Rect& from) {
        rj_geometry_msgs::msg::Rect to;
        convert_to_ros(from.pt, &to.pt);
        return to;
    }

    static rj_geometry::Rect from_ros(const rj_geometry_msgs::msg::Rect& from) {
        rj_geometry::Rect to;
        convert_from_ros(from.pt, &to.pt);
        return to;
    }
};

ASSOCIATE_CPP_ROS(rj_geometry::Rect, rj_geometry::Rect::Msg);

template <>
struct RosConverter<rj_geometry::Circle, rj_geometry_msgs::msg::Circle> {
    static rj_geometry_msgs::msg::Circle to_ros(const rj_geometry::Circle& from) {
        return rj_geometry_msgs::build<rj_geometry_msgs::msg::Circle>().center(convert_to_ros(from.center)).radius(convert_to_ros(from.radius()));
    }

    static rj_geometry::Circle from_ros(const rj_geometry_msgs::msg::Circle& from) {
        return rj_geometry::Circle(rj_convert::convert_from_ros(from.center), from.radius);
    }
};

ASSOCIATE_CPP_ROS(rj_geometry::Circle, rj_geometry::Circle::Msg);

template <>
struct RosConverter<rj_geometry::Polygon, rj_geometry_msgs::msg::Polygon> {
    static rj_geometry_msgs::msg::Polygon to_ros(const rj_geometry::Polygon& from) {
        return rj_geometry_msgs::build<rj_geometry_msgs::msg::Polygon>().points(convert_to_ros(from.vertices));
    }

    static rj_geometry::Polygon from_ros(const rj_geometry_msgs::msg::Polygon& from) {
        return rj_geometry::Polygon(convert_from_ros(from.points));
    }
};

ASSOCIATE_CPP_ROS(rj_geometry::Polygon, rj_geometry::Polygon::Msg);

template <>
struct RosConverter<rj_geometry::ShapeSet, rj_geometry_msgs::msg::ShapeSet> {
    static rj_geometry_msgs::msg::ShapeSet to_ros(const rj_geometry::ShapeSet& from) {
        rj_geometry_msgs::msg::ShapeSet result;
        // This is definitely a bit sketchy. We have to explicitly check each shape's type.
        for (const auto& shape : from.shapes()) {
            if (const auto* as_rect = dynamic_cast<rj_geometry::Rect*>(shape.get())) {
                result.rectangles.emplace_back(convert_to_ros(*as_rect));
            } else if (const auto* as_circle = dynamic_cast<rj_geometry::Circle*>(shape.get())) {
                result.circles.emplace_back(convert_to_ros(*as_circle));
            } else if (const auto* as_polygon = dynamic_cast<rj_geometry::Polygon*>(shape.get())) {
                result.polygons.emplace_back(convert_to_ros(*as_polygon));
            } else {
                throw std::invalid_argument("Object in ShapeSet has invalid type in conversion to ROS type");
            }
        }
        return result;
    }

    static rj_geometry::ShapeSet from_ros(const rj_geometry_msgs::msg::ShapeSet& from) {
        rj_geometry::ShapeSet result;
        std::transform(from.rectangles.begin(),
                       from.rectangles.end(),
                       std::back_inserter(result.shapes()),
                       [] (const auto &rect) { return std::make_shared<rj_geometry::Rect>(convert_from_ros(rect)); });
        std::transform(from.circles.begin(),
                       from.circles.end(),
                       std::back_inserter(result.shapes()),
                       [] (const auto &rect) { return std::make_shared<rj_geometry::Circle>(convert_from_ros(rect)); });
        std::transform(from.polygons.begin(),
                       from.polygons.end(),
                       std::back_inserter(result.shapes()),
                       [] (const auto &rect) { return std::make_shared<rj_geometry::Polygon>(convert_from_ros(rect)); });
        return result;
    }
};

ASSOCIATE_CPP_ROS(rj_geometry::ShapeSet, rj_geometry::ShapeSet::Msg);

}  // namespace rj_convert