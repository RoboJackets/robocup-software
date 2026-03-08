/**
 * @file mark_robot.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief Extensions on the Mark Robot Action message in rj_control_msgs
 * @version 0.1
 * @date 2026-01-04
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <rj_convert/ros_convert.hpp>
#include <rj_control_msgs/msg/mark_robot.hpp>

namespace action {

/**
 * @brief The mark robot action is called to position this robot a given distance between an opponent's
 * robot and the goal
 * 
 */
class MarkRobot {
public:
    using Msg = rj_control_msgs::msg::MarkRobot;

    /**
     * @brief Construct a new Mark Robot Action
     * 
     * @param their_robot_id The opposing robot id to mark
     * @param distance The distance from the robot to mark
     */
    //NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    MarkRobot(int their_robot_id, double distance)
        : their_robot_id_(their_robot_id), distance_(distance) {}

    /**
     * @brief Get the id of the robot to mark
     * 
     * @return int 
     */
    [[nodiscard]] int their_robot_id() const {
        return their_robot_id_;
    }

    /**
     * @brief Set the id of the robot to mark
     * 
     * @param their_robot_id 
     */
    void set_their_robot_id(int their_robot_id) {
        their_robot_id_ = their_robot_id;
    }

    /**
     * @brief Get the distance from the robot to mark at
     * 
     * @return double 
     */
    [[nodiscard]] double distance() const {
        return distance_;
    }

    /**
     * @brief Set the distance from the robot to mark at
     * 
     * @param distance 
     */
    void set_distance(double distance) {
        distance_ = distance;
    }

private:
    // The robot id of the opposing robot to mark
    int their_robot_id_;
    // The distance from the opposing robot to stay
    double distance_;
};

} // namespace action

namespace rj_convert {

template <>
struct RosConverter<action::MarkRobot, action::MarkRobot::Msg> {
    static rj_control_msgs::msg::MarkRobot to_ros(const action::MarkRobot& from) {
        rj_control_msgs::msg::MarkRobot msg;
        msg.their_robot_id = from.their_robot_id();
        msg.distance = from.distance();
        return msg;
    }

    static action::MarkRobot from_ros(const rj_control_msgs::msg::MarkRobot& from) {
        return {
            from.their_robot_id,
            from.distance
        };
    }
};

ASSOCIATE_CPP_ROS(action::MarkRobot, action::MarkRobot::Msg);

} // namespace rj_convert