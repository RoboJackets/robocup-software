#pragma once

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <vector>


#include <rclcpp/type_adapter.hpp>

#include <rj_geometry/circle.hpp>
#include <rj_geometry/line.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry/polygon.hpp>
#include <rj_geometry/pose.hpp>
#include <rj_geometry/rect.hpp>
#include <rj_geometry/segment.hpp>
#include <rj_geometry/shape_set.hpp>
#include <rj_geometry_msgs/msg/line.hpp>
#include <rj_geometry_msgs/msg/point.hpp>
#include <rj_geometry_msgs/msg/polygon.hpp>
#include <rj_geometry_msgs/msg/pose.hpp>
#include <rj_geometry_msgs/msg/rect.hpp>
#include <rj_geometry_msgs/msg/segment.hpp>
#include <rj_geometry_msgs/msg/shape_set.hpp>
#include <rj_geometry_msgs/msg/twist.hpp>

namespace rclcpp {

template <>
struct TypeAdapter<rj_geometry::Point, rj_geometry_msgs::msg::Point> {
    using is_specialized = std::true_type;
    using custom_type = rj_geometry::Point;
    using ros_message_type = rj_geometry_msgs::msg::Point;

    static void convert_to_ros_message(const custom_type& source, ros_message_type& destination) {
        destination.x = source.x();
        destination.y = source.y();
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination.x() = source.x;
        destination.y() = source.y;
    }
};

template <>
struct TypeAdapter<rj_geometry::Pose, rj_geometry_msgs::msg::Pose> {
    using is_specialized = std::true_type;
    using custom_type = rj_geometry::Pose;
    using ros_message_type = rj_geometry_msgs::msg::Pose;

    static void convert_to_ros_message(const custom_type& source, ros_message_type& destination) {
        rclcpp::TypeAdapter<rj_geometry::Point, rj_geometry_msgs::msg::Point>::convert_to_ros_message(
            source.position(), destination.position);
        destination.heading = source.heading();
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        rclcpp::TypeAdapter<rj_geometry::Point, rj_geometry_msgs::msg::Point>::convert_to_custom(
            source.position, destination.position());
        destination.heading() = source.heading;
    }
};

template <>
struct TypeAdapter<rj_geometry::Twist, rj_geometry_msgs::msg::Twist> {
    using is_specialized = std::true_type;
    using custom_type = rj_geometry::Twist;
    using ros_message_type = rj_geometry_msgs::msg::Twist;

    static void convert_to_ros_message(const custom_type& source, ros_message_type& destination) {
        rclcpp::TypeAdapter<rj_geometry::Point, rj_geometry_msgs::msg::Point>::convert_to_ros_message(
            source.linear(), destination.linear);
        destination.angular = source.angular();
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        rclcpp::TypeAdapter<rj_geometry::Point, rj_geometry_msgs::msg::Point>::convert_to_custom(
            source.linear, destination.linear());
        destination.angular() = source.angular;
    }
};

template <>
struct TypeAdapter<rj_geometry::Line, rj_geometry_msgs::msg::Line> {
    using is_specialized = std::true_type;
    using custom_type = rj_geometry::Line;
    using ros_message_type = rj_geometry_msgs::msg::Line;

    static void convert_to_ros_message(const custom_type& source, ros_message_type& destination) {
        for (size_t i = 0; i < source.pt.size(); ++i) {
            rclcpp::TypeAdapter<rj_geometry::Point, rj_geometry_msgs::msg::Point>::
                convert_to_ros_message(source.pt[i], destination.pt[i]);
        }
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        for (size_t i = 0; i < source.pt.size(); ++i) {
            rclcpp::TypeAdapter<rj_geometry::Point, rj_geometry_msgs::msg::Point>::convert_to_custom(
                source.pt[i], destination.pt[i]);
        }
    }
};

template <>
struct TypeAdapter<rj_geometry::Segment, rj_geometry_msgs::msg::Segment> {
    using is_specialized = std::true_type;
    using custom_type = rj_geometry::Segment;
    using ros_message_type = rj_geometry_msgs::msg::Segment;

    static void convert_to_ros_message(const custom_type& source, ros_message_type& destination) {
        for (size_t i = 0; i < source.pt.size(); ++i) {
            rclcpp::TypeAdapter<rj_geometry::Point, rj_geometry_msgs::msg::Point>::
                convert_to_ros_message(source.pt[i], destination.pt[i]);
        }
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        for (size_t i = 0; i < source.pt.size(); ++i) {
            rclcpp::TypeAdapter<rj_geometry::Point, rj_geometry_msgs::msg::Point>::convert_to_custom(
                source.pt[i], destination.pt[i]);
        }
    }
};

template <>
struct TypeAdapter<rj_geometry::Rect, rj_geometry_msgs::msg::Rect> {
    using is_specialized = std::true_type;
    using custom_type = rj_geometry::Rect;
    using ros_message_type = rj_geometry_msgs::msg::Rect;

    static void convert_to_ros_message(const custom_type& source, ros_message_type& destination) {
        for (size_t i = 0; i < source.pt.size(); ++i) {
            rclcpp::TypeAdapter<rj_geometry::Point, rj_geometry_msgs::msg::Point>::
                convert_to_ros_message(source.pt[i], destination.pt[i]);
        }
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        for (size_t i = 0; i < source.pt.size(); ++i) {
            rclcpp::TypeAdapter<rj_geometry::Point, rj_geometry_msgs::msg::Point>::convert_to_custom(
                source.pt[i], destination.pt[i]);
        }
    }
};

template <>
struct TypeAdapter<rj_geometry::Circle, rj_geometry_msgs::msg::Circle> {
    using is_specialized = std::true_type;
    using custom_type = rj_geometry::Circle;
    using ros_message_type = rj_geometry_msgs::msg::Circle;

    static void convert_to_ros_message(const custom_type& source, ros_message_type& destination) {
        rj_geometry_msgs::msg::Point center;
        rclcpp::TypeAdapter<rj_geometry::Point, rj_geometry_msgs::msg::Point>::convert_to_ros_message(
            source.center, center);
        destination = rj_geometry_msgs::build<rj_geometry_msgs::msg::Circle>()
                          .center(center)
                          .radius(source.radius());
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        rj_geometry::Point center;
        rclcpp::TypeAdapter<rj_geometry::Point, rj_geometry_msgs::msg::Point>::convert_to_custom(
            source.center, center);
        destination = rj_geometry::Circle(center, source.radius);
    }
};

template <>
struct TypeAdapter<rj_geometry::Polygon, rj_geometry_msgs::msg::Polygon> {
    using is_specialized = std::true_type;
    using custom_type = rj_geometry::Polygon;
    using ros_message_type = rj_geometry_msgs::msg::Polygon;

    static void convert_to_ros_message(const custom_type& source, ros_message_type& destination) {
        destination.points.clear();
        destination.points.reserve(source.vertices.size());
        for (const auto& vertex : source.vertices) {
            rj_geometry_msgs::msg::Point point;
            rclcpp::TypeAdapter<rj_geometry::Point,
                                rj_geometry_msgs::msg::Point>::convert_to_ros_message(vertex,
                                                                                      point);
            destination.points.push_back(std::move(point));
        }
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination.vertices.clear();
        destination.vertices.reserve(source.points.size());
        for (const auto& point : source.points) {
            rj_geometry::Point vertex;
            rclcpp::TypeAdapter<rj_geometry::Point,
                                rj_geometry_msgs::msg::Point>::convert_to_custom(point, vertex);
            destination.vertices.push_back(std::move(vertex));
        }
    }
};

template <>
struct TypeAdapter<rj_geometry::ShapeSet, rj_geometry_msgs::msg::ShapeSet> {
    using is_specialized = std::true_type;
    using custom_type = rj_geometry::ShapeSet;
    using ros_message_type = rj_geometry_msgs::msg::ShapeSet;

    static void convert_to_ros_message(const custom_type& source, ros_message_type& destination) {
        destination = ros_message_type{};
        // This is definitely a bit sketchy. We have to explicitly check each shape's type.
        for (const auto& shape : source.shapes()) {
            if (const auto* as_rect = dynamic_cast<rj_geometry::Rect*>(shape.get())) {
                rj_geometry_msgs::msg::Rect rect;
                rclcpp::TypeAdapter<rj_geometry::Rect, rj_geometry_msgs::msg::Rect>::
                    convert_to_ros_message(*as_rect, rect);
                destination.rectangles.emplace_back(std::move(rect));
            } else if (const auto* as_circle = dynamic_cast<rj_geometry::Circle*>(shape.get())) {
                rj_geometry_msgs::msg::Circle circle;
                rclcpp::TypeAdapter<rj_geometry::Circle, rj_geometry_msgs::msg::Circle>::
                    convert_to_ros_message(*as_circle, circle);
                destination.circles.emplace_back(std::move(circle));
            } else if (const auto* as_polygon = dynamic_cast<rj_geometry::Polygon*>(shape.get())) {
                rj_geometry_msgs::msg::Polygon polygon;
                rclcpp::TypeAdapter<rj_geometry::Polygon, rj_geometry_msgs::msg::Polygon>::
                    convert_to_ros_message(*as_polygon, polygon);
                destination.polygons.emplace_back(std::move(polygon));
            } else {
                throw std::invalid_argument(
                    "Object in ShapeSet has invalid type in conversion to ROS type");
            }
        }
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = custom_type{};
        std::transform(
            source.rectangles.begin(), source.rectangles.end(),
            std::back_inserter(destination.shapes()), [](const auto& rect) {
                rj_geometry::Rect converted;
                rclcpp::TypeAdapter<rj_geometry::Rect, rj_geometry_msgs::msg::Rect>::convert_to_custom(
                    rect, converted);
                return std::make_shared<rj_geometry::Rect>(std::move(converted));
            });
        std::transform(source.circles.begin(), source.circles.end(),
                       std::back_inserter(destination.shapes()), [](const auto& circle) {
                           rj_geometry::Circle converted;
                           rclcpp::TypeAdapter<rj_geometry::Circle, rj_geometry_msgs::msg::Circle>::
                               convert_to_custom(circle, converted);
                           return std::make_shared<rj_geometry::Circle>(std::move(converted));
                       });
        std::transform(source.polygons.begin(), source.polygons.end(),
                       std::back_inserter(destination.shapes()), [](const auto& polygon) {
                           rj_geometry::Polygon converted;
                           rclcpp::TypeAdapter<rj_geometry::Polygon,
                                               rj_geometry_msgs::msg::Polygon>::convert_to_custom(
                               polygon, converted);
                           return std::make_shared<rj_geometry::Polygon>(std::move(converted));
                       });
    }
};

}  // namespace rclcpp