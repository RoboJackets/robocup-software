#include "debug_draw_interface.hpp"

#include <spdlog/spdlog.h>

#include <rj_constants/topic_names.hpp>

namespace ros2_temp {

DebugDrawInterface::DebugDrawInterface(Context* context, rclcpp::Executor* executor)
    : context_(context) {
    node_ = std::make_shared<rclcpp::Node>("_debug_draw_interface");
    debug_draw_sub_ = node_->create_subscription<rj_drawing_msgs::msg::DebugDraw>(
        viz::topics::kDebugDrawPub, rclcpp::QoS(10),
        [this](rj_drawing_msgs::msg::DebugDraw::SharedPtr debug_draw) {  // NOLINT
            spdlog::info("Got message");
            latest_[debug_draw->layer] = debug_draw;
        });

    executor->add_node(node_);
}

void DebugDrawInterface::run() {
    const auto& color_to_qt = [](const rj_drawing_msgs::msg::DrawColor& color) {
        return QColor::fromRgb(color.r, color.g, color.b);
    };

    for (const auto& [layer, debug_draw] : latest_) {
        spdlog::info("Got layer {}", layer);
        for (const auto& shapes : debug_draw->shapes) {
            context_->debug_drawer.draw_shape_set(rj_convert::convert_from_ros(shapes.shapes),
                                                  color_to_qt(shapes.color),
                                                  QString::fromStdString(layer));
        }
        for (const auto& segment : debug_draw->segments) {
            context_->debug_drawer.draw_segment(rj_convert::convert_from_ros(segment.segment),
                                                color_to_qt(segment.color),
                                                QString::fromStdString(layer));
        }
        for (const auto& pose : debug_draw->poses) {
            // TODO(#1584): Handle poses
        }
        for (const auto& path : debug_draw->paths) {
            auto* debug_path = context_->debug_drawer.add_debug_path();
            for (const auto& point : path.points) {
                auto* new_point = debug_path->add_points();
                new_point->mutable_pos()->set_x(point.x);
                new_point->mutable_pos()->set_y(point.y);

                // TODO(Kyle): Use color in trajectory
            }
        }
        for (const auto& text : debug_draw->debug_text) {
            context_->debug_drawer.draw_text(
                QString::fromStdString(text.text), rj_convert::convert_from_ros(text.position),
                color_to_qt(text.color), QString::fromStdString(layer));
        }
    }
}

}  // namespace ros2_temp