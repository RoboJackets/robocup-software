#pragma once

#include <algorithm>

#include <rj_convert/ros_convert.hpp>
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

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        rj_convert::convert_to_ros(source.x(), &destination.x);
        rj_convert::convert_to_ros(source.y(), &destination.y);
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        rj_convert::convert_from_ros(source.x, &destination.x());
        rj_convert::convert_from_ros(source.y, &destination.y());
    }
};


template <>
struct TypeAdapter<rj_geometry::Pose, rj_geometry_msgs::msg::Pose> {
    using is_specialized = std::true_type;
    using custom_type = rj_geometry::Pose;
    using ros_message_type = rj_geometry_msgs::msg::Pose;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        rj_convert::convert_to_ros(source.position(), &destination.position);
        rj_convert::convert_to_ros(source.heading(), &destination.heading);
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        rj_convert::convert_from_ros(source.position, &destination.position());
        rj_convert::convert_from_ros(source.heading, &destination.heading());
    }
};


template <>
struct TypeAdapter<rj_geometry::Twist, rj_geometry_msgs::msg::Twist> {
    using is_specialized = std::true_type;
    using custom_type = rj_geometry::Twist;
    using ros_message_type = rj_geometry_msgs::msg::Twist;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        rj_convert::convert_to_ros(source.linear(), &destination.linear);
        rj_convert::convert_to_ros(source.angular(), &destination.angular);
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        rj_convert::convert_from_ros(source.linear, &destination.linear());
        rj_convert::convert_from_ros(source.angular, &destination.angular());
    }
};


template <>
struct TypeAdapter<rj_geometry::Line, rj_geometry_msgs::msg::Line> {
    using is_specialized = std::true_type;
    using custom_type = rj_geometry::Line;
    using ros_message_type = rj_geometry_msgs::msg::Line;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        for (size_t i = 0; i < source.pt.size(); ++i) {
            rj_convert::convert_to_ros(source.pt[i], &destination.pt[i]);
        }
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        for (size_t i = 0; i < source.pt.size(); ++i) {
            rj_convert::convert_from_ros(source.pt[i], &destination.pt[i]);
        }
    }
};


template <>
struct TypeAdapter<rj_geometry::Segment, rj_geometry_msgs::msg::Segment> {
    using is_specialized = std::true_type;
    using custom_type = rj_geometry::Segment;
    using ros_message_type = rj_geometry_msgs::msg::Segment;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        for (size_t i = 0; i < source.pt.size(); ++i) {
            rj_convert::convert_to_ros(source.pt[i], &destination.pt[i]);
        }
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        for (size_t i = 0; i < source.pt.size(); ++i) {
            rj_convert::convert_from_ros(source.pt[i], &destination.pt[i]);
        }
    }
};


template <>
struct TypeAdapter<rj_geometry::Rect, rj_geometry_msgs::msg::Rect> {
    using is_specialized = std::true_type;
    using custom_type = rj_geometry::Rect;
    using ros_message_type = rj_geometry_msgs::msg::Rect;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        for (size_t i = 0; i < source.pt.size(); ++i) {
            rj_convert::convert_to_ros(source.pt[i], &destination.pt[i]);
        }
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        for (size_t i = 0; i < source.pt.size(); ++i) {
            rj_convert::convert_from_ros(source.pt[i], &destination.pt[i]);
        }
    }
};


template <>
struct TypeAdapter<rj_geometry::Circle, rj_geometry_msgs::msg::Circle> {
    using is_specialized = std::true_type;
    using custom_type = rj_geometry::Circle;
    using ros_message_type = rj_geometry_msgs::msg::Circle;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination = rj_geometry_msgs::build<rj_geometry_msgs::msg::Circle>()
                          .center(rj_convert::convert_to_ros<rj_geometry::Point,
                                                             rj_geometry_msgs::msg::Point>(
                              source.center))
                          .radius(source.radius());
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = rj_geometry::Circle(
            rj_convert::convert_from_ros<rj_geometry_msgs::msg::Point, rj_geometry::Point>(
                source.center),
            source.radius);
    }
};


template <>
struct TypeAdapter<rj_geometry::Polygon, rj_geometry_msgs::msg::Polygon> {
    using is_specialized = std::true_type;
    using custom_type = rj_geometry::Polygon;
    using ros_message_type = rj_geometry_msgs::msg::Polygon;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination = rj_geometry_msgs::build<rj_geometry_msgs::msg::Polygon>().points(
            rj_convert::convert_to_ros<std::vector<rj_geometry::Point>,
                                       std::vector<rj_geometry_msgs::msg::Point>>(
                source.vertices));
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = rj_geometry::Polygon(rj_convert::convert_from_ros<
                                    std::vector<rj_geometry_msgs::msg::Point>,
                                    std::vector<rj_geometry::Point>>(source.points));
    }
};


template <>
struct TypeAdapter<rj_geometry::ShapeSet, rj_geometry_msgs::msg::ShapeSet> {
    using is_specialized = std::true_type;
    using custom_type = rj_geometry::ShapeSet;
    using ros_message_type = rj_geometry_msgs::msg::ShapeSet;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination = ros_message_type{};
        // This is definitely a bit sketchy. We have to explicitly check each shape's type.
        for (const auto& shape : source.shapes()) {
            if (const auto* as_rect = dynamic_cast<rj_geometry::Rect*>(shape.get())) {
                destination.rectangles.emplace_back(
                    rj_convert::convert_to_ros<rj_geometry::Rect,
                                               rj_geometry_msgs::msg::Rect>(*as_rect));
            } else if (const auto* as_circle = dynamic_cast<rj_geometry::Circle*>(shape.get())) {
                destination.circles.emplace_back(
                    rj_convert::convert_to_ros<rj_geometry::Circle,
                                               rj_geometry_msgs::msg::Circle>(*as_circle));
            } else if (const auto* as_polygon = dynamic_cast<rj_geometry::Polygon*>(shape.get())) {
                destination.polygons.emplace_back(
                    rj_convert::convert_to_ros<rj_geometry::Polygon,
                                               rj_geometry_msgs::msg::Polygon>(*as_polygon));
            } else {
                throw std::invalid_argument(
                    "Object in ShapeSet has invalid type in conversion to ROS type");
            }
        }
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = custom_type{};
        std::transform(source.rectangles.begin(), source.rectangles.end(),
                       std::back_inserter(destination.shapes()), [](const auto& rect) {
                           return std::make_shared<rj_geometry::Rect>(
                               rj_convert::convert_from_ros<rj_geometry_msgs::msg::Rect,
                                                            rj_geometry::Rect>(rect));
                       });
        std::transform(source.circles.begin(), source.circles.end(),
                       std::back_inserter(destination.shapes()), [](const auto& rect) {
                           return std::make_shared<rj_geometry::Circle>(
                               rj_convert::convert_from_ros<rj_geometry_msgs::msg::Circle,
                                                            rj_geometry::Circle>(rect));
                       });
        std::transform(source.polygons.begin(), source.polygons.end(),
                       std::back_inserter(destination.shapes()), [](const auto& rect) {
                           return std::make_shared<rj_geometry::Polygon>(
                               rj_convert::convert_from_ros<rj_geometry_msgs::msg::Polygon,
                                                            rj_geometry::Polygon>(rect));
                       });
    }
};


}  // namespace rclcpp