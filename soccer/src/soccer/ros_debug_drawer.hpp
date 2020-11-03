#pragma once

#include <QColor>
#include <rclcpp/rclcpp.hpp>

#include <rj_convert/ros_convert.hpp>
#include <rj_drawing_msgs/msg/debug_draw.hpp>
#include <rj_drawing_msgs/msg/draw_color.hpp>
#include <rj_drawing_msgs/msg/draw_path.hpp>
#include <rj_drawing_msgs/msg/draw_pose.hpp>
#include <rj_drawing_msgs/msg/draw_segment.hpp>
#include <rj_drawing_msgs/msg/draw_shapes.hpp>
#include <rj_drawing_msgs/msg/draw_text.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>

namespace rj_drawing {

class RosDebugDrawer {
public:
    RosDebugDrawer(rclcpp::Publisher<rj_drawing_msgs::msg::DebugDraw>::SharedPtr pub,
                   const std::string& layer)
        : pub_(std::move(pub)), layer_(layer) {
        frame_.layer = layer_;
    }

    void draw_shapes(const rj_geometry::ShapeSet& shapes, QColor color = QColor::fromRgb(0, 0, 0)) {
        frame_.shapes.push_back(rj_drawing_msgs::build<rj_drawing_msgs::msg::DrawShapes>()
                                    .shapes(rj_convert::convert_to_ros(shapes))
                                    .color(color_from_qt(color)));
    }

    void draw_circle(const rj_geometry::Circle& circle, QColor color = QColor::fromRgb(0, 0, 0)) {
        rj_geometry_msgs::msg::ShapeSet shapes;
        shapes.circles.push_back(rj_convert::convert_to_ros(circle));
        frame_.shapes.push_back(
            rj_drawing_msgs::build<rj_drawing_msgs::msg::DrawShapes>().shapes(shapes).color(
                color_from_qt(color)));
    }

    void draw_rect(const rj_geometry::Rect& rect, QColor color = QColor::fromRgb(0, 0, 0)) {
        rj_geometry_msgs::msg::ShapeSet shapes;
        shapes.rectangles.push_back(rj_convert::convert_to_ros(rect));
        frame_.shapes.push_back(
            rj_drawing_msgs::build<rj_drawing_msgs::msg::DrawShapes>().shapes(shapes).color(
                color_from_qt(color)));
    }

    void draw_polygon(const rj_geometry::Polygon& polygon,
                      QColor color = QColor::fromRgb(0, 0, 0)) {
        rj_geometry_msgs::msg::ShapeSet shapes;
        shapes.polygons.push_back(rj_convert::convert_to_ros(polygon));
        frame_.shapes.push_back(
            rj_drawing_msgs::build<rj_drawing_msgs::msg::DrawShapes>().shapes(shapes).color(
                color_from_qt(color)));
    }

    void draw_segment(const rj_geometry::Segment& segment,
                      QColor color = QColor::fromRgb(0, 0, 0)) {
        frame_.segments.push_back(rj_drawing_msgs::build<rj_drawing_msgs::msg::DrawSegment>()
                                      .segment(rj_convert::convert_to_ros(segment))
                                      .color(color_from_qt(color)));
    }

    void draw_pose(const rj_geometry::Pose& pose, QColor color = QColor::fromRgb(0, 0, 0)) {
        frame_.poses.push_back(rj_drawing_msgs::build<rj_drawing_msgs::msg::DrawPose>()
                                   .pose(rj_convert::convert_to_ros(pose))
                                   .color(color_from_qt(color)));
    }

    void draw_text(const std::string& text, const rj_geometry::Point& position,
                   QColor color = QColor::fromRgb(0, 0, 0)) {
        frame_.debug_text.push_back(rj_drawing_msgs::build<rj_drawing_msgs::msg::DrawText>()
                                        .text(text)
                                        .position(rj_convert::convert_to_ros(position))
                                        .color(color_from_qt(color)));
    }

    void publish() {
        pub_->publish(frame_);
        frame_ = rj_drawing_msgs::msg::DebugDraw{};
        frame_.layer = layer_;
    }

private:
    static rj_drawing_msgs::msg::DrawColor color_from_qt(QColor color) {
        return rj_drawing_msgs::build<rj_drawing_msgs::msg::DrawColor>()
            .r(color.red())
            .g(color.green())
            .b(color.blue())
            .a(color.alpha());
    }

    rclcpp::Publisher<rj_drawing_msgs::msg::DebugDraw>::SharedPtr pub_;
    std::string layer_;

    rj_drawing_msgs::msg::DebugDraw frame_;
};

}  // namespace rj_drawing
