#pragma once

#include <QColor>
#include <rclcpp/rclcpp.hpp>


#include <rj_drawing_msgs/msg/debug_draw.hpp>
#include <rj_drawing_msgs/msg/draw_color.hpp>
#include <rj_drawing_msgs/msg/draw_path.hpp>
#include <rj_drawing_msgs/msg/draw_pose.hpp>
#include <rj_drawing_msgs/msg/draw_segment.hpp>
#include <rj_drawing_msgs/msg/draw_shapes.hpp>
#include <rj_drawing_msgs/msg/draw_text.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry/stadium_shape.hpp>

namespace rj_drawing {

class RosDebugDrawer {
public:
    RosDebugDrawer(rclcpp::Publisher<rj_drawing_msgs::msg::DebugDraw>::SharedPtr pub,
                   std::string layer)
        : pub_(std::move(pub)), layer_(std::move(layer)) {
        frame_.layer = layer_;
    }

    void draw_shapes(const rj_geometry::ShapeSet& shapes,
                     const QColor& color = QColor::fromRgb(0, 0, 0, 0)) {
        rj_geometry_msgs::msg::ShapeSet shapes_msg;
        rclcpp::TypeAdapter<rj_geometry::ShapeSet, rj_geometry_msgs::msg::ShapeSet>::
            convert_to_ros_message(shapes, shapes_msg);
        frame_.shapes.push_back(rj_drawing_msgs::build<rj_drawing_msgs::msg::DrawShapes>()
                                    .shapes(shapes_msg)
                                    .color(color_from_qt(color)));
    }

    void draw_circle(const rj_geometry::Circle& circle,
                     const QColor& color = QColor::fromRgb(0, 0, 0, 0)) {
        rj_geometry_msgs::msg::ShapeSet shapes;
        rj_geometry_msgs::msg::Circle circle_msg;
        rclcpp::TypeAdapter<rj_geometry::Circle, rj_geometry_msgs::msg::Circle>::
            convert_to_ros_message(circle, circle_msg);
        shapes.circles.push_back(circle_msg);
        frame_.shapes.push_back(
            rj_drawing_msgs::build<rj_drawing_msgs::msg::DrawShapes>().shapes(shapes).color(
                color_from_qt(color)));
    }

    void draw_rect(const rj_geometry::Rect& rect,
                   const QColor& color = QColor::fromRgb(0, 0, 0, 0)) {
        rj_geometry_msgs::msg::ShapeSet shapes;
        rj_geometry_msgs::msg::Rect rect_msg;
        rclcpp::TypeAdapter<rj_geometry::Rect, rj_geometry_msgs::msg::Rect>::convert_to_ros_message(
            rect, rect_msg);
        shapes.rectangles.push_back(rect_msg);
        frame_.shapes.push_back(
            rj_drawing_msgs::build<rj_drawing_msgs::msg::DrawShapes>().shapes(shapes).color(
                color_from_qt(color)));
    }

    void draw_polygon(const rj_geometry::Polygon& polygon,
                      const QColor& color = QColor::fromRgb(0, 0, 0, 0)) {
        rj_geometry_msgs::msg::ShapeSet shapes;
        rj_geometry_msgs::msg::Polygon polygon_msg;
        rclcpp::TypeAdapter<rj_geometry::Polygon, rj_geometry_msgs::msg::Polygon>::
            convert_to_ros_message(polygon, polygon_msg);
        shapes.polygons.push_back(polygon_msg);
        frame_.shapes.push_back(
            rj_drawing_msgs::build<rj_drawing_msgs::msg::DrawShapes>().shapes(shapes).color(
                color_from_qt(color)));
    }

    void draw_stadium(const rj_geometry::StadiumShape& stadium,
                      const QColor& color = QColor::fromRgb(0, 0, 0, 0)) {
        this->draw_shapes(stadium.drawshapes(), color);
    }

    void draw_segment(const rj_geometry::Segment& segment,
                      const QColor& color = QColor::fromRgb(0, 0, 0)) {
        rj_geometry_msgs::msg::Segment segment_msg;
        rclcpp::TypeAdapter<rj_geometry::Segment, rj_geometry_msgs::msg::Segment>::
            convert_to_ros_message(segment, segment_msg);
        frame_.segments.push_back(rj_drawing_msgs::build<rj_drawing_msgs::msg::DrawSegment>()
                                      .segment(segment_msg)
                                      .color(color_from_qt(color)));
    }

    void draw_pose(const rj_geometry::Pose& pose, const QColor& color = QColor::fromRgb(0, 0, 0)) {
        rj_geometry_msgs::msg::Pose pose_msg;
        rclcpp::TypeAdapter<rj_geometry::Pose, rj_geometry_msgs::msg::Pose>::convert_to_ros_message(
            pose, pose_msg);
        frame_.poses.push_back(rj_drawing_msgs::build<rj_drawing_msgs::msg::DrawPose>()
                                   .pose(pose_msg)
                                   .color(color_from_qt(color)));
    }

    void draw_text(const std::string& text, const rj_geometry::Point& position,
                   const QColor& color = QColor::fromRgb(0, 0, 0)) {
        rj_geometry_msgs::msg::Point position_msg;
        rclcpp::TypeAdapter<rj_geometry::Point, rj_geometry_msgs::msg::Point>::convert_to_ros_message(
            position, position_msg);
        frame_.debug_text.push_back(rj_drawing_msgs::build<rj_drawing_msgs::msg::DrawText>()
                                        .text(text)
                                        .position(position_msg)
                                        .color(color_from_qt(color)));
    }

    void draw_path(const std::vector<rj_geometry::Point>& points) {
        frame_.paths.emplace_back();
        for (const auto& point : points) {
            rj_geometry_msgs::msg::Point point_msg;
            rclcpp::TypeAdapter<rj_geometry::Point, rj_geometry_msgs::msg::Point>::
                convert_to_ros_message(point, point_msg);
            frame_.paths.back().points.push_back(point_msg);
        }
    }

    void publish() {
        pub_->publish(frame_);
        frame_ = rj_drawing_msgs::msg::DebugDraw{};
        frame_.layer = layer_;
    }

private:
    static rj_drawing_msgs::msg::DrawColor color_from_qt(const QColor& color) {
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
