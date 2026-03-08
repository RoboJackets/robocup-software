/**
 * @file collect.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief Extensions on the Collect Action message in rj_control_msgs
 * @version 0.1
 * @date 2026-01-04
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <rj_control_msgs/msg/collect.hpp>
#include <rj_convert/ros_convert.hpp>

namespace action {

/**
 * @brief The collect action has the robot run to the ball and turn on the dribbler
 * 
 */
class Collect {
public:
    using Msg = rj_control_msgs::msg::Collect;    

    /**
     * @brief Construct a new Collect Action
     * 
     */
    Collect() = default;
};

} // namespace action

namespace rj_convert {

template <>
struct RosConverter<action::Collect, rj_control_msgs::msg::Collect> {
    static rj_control_msgs::msg::Collect to_ros([[maybe_unused]] const action::Collect& from) {
        rj_control_msgs::msg::Collect collect;
        return collect;
    }

    static action::Collect from_ros([[maybe_unused]] const rj_control_msgs::msg::Collect& from) {
        return {};
    }
};

ASSOCIATE_CPP_ROS(action::Collect, action::Collect::Msg);

} // namespace rj_convert