/**
 * @file line_test_node.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief The Line Test Node moves robots in horizontal lines across the field
 * @version 0.1
 * @date 2026-01-11
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <rj_utils/logging.hpp>
#include <rj_utils/parsing.hpp>
#include <rj_common/field_dimensions.hpp>
#include <rj_control_extensions/action.hpp>
#include <rj_geometry/point.hpp>

namespace rj_testing {

class LineTestNode : public rclcpp::Node {
public:
    /**
     * @brief Construct a new Line Test Node
     * 
     */
    LineTestNode();

private:
    /**
     * @brief Determine where the robot should go
     * 
     * @return rj_geometry::Point 
     */
    rj_geometry::Point calculate_desired_target();

    // The direction to be moving
    enum Direction {
        // Moving Left
        LEFT = 0,
        // Moving Right
        RIGHT = 1,
    };

    // The robot id the robot being controlled
    int robot_id_;
    // The current direction to move
    Direction direction_ = Direction::LEFT;
    // The field dimensions
    FieldDimensions field_dimensions_ = FieldDimensions::kDefaultDimensions;

    // Publisher to publish the move action
    std::shared_ptr<rclcpp::Publisher<action::Action::Msg>> action_pub_;
    // Subscription to the action complete message
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> action_complete_sub_;
    // Subscription to the field dimensions
    std::shared_ptr<rclcpp::Subscription<FieldDimensions::Msg>> field_dimensions_sub_;
};

} // namespace rj_testing