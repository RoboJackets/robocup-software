/**
 * @file action.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief Extensions on the Action message in rj_control_msgs
 * @version 0.1
 * @date 2026-01-03
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <optional>

#include <rj_convert/ros_convert.hpp>
#include <rj_control_msgs/msg/action.hpp>

#include "rj_control_extensions/actions/clear.hpp"
#include "rj_control_extensions/actions/collect.hpp"
#include "rj_control_extensions/actions/dribble.hpp"
#include "rj_control_extensions/actions/go_to_point.hpp"
#include "rj_control_extensions/actions/go_to_pose.hpp"
#include "rj_control_extensions/actions/mark_robot.hpp"
#include "rj_control_extensions/actions/pass.hpp"
#include "rj_control_extensions/actions/shoot.hpp"
#include "rj_control_extensions/actions/rotate_to_heading.hpp"


namespace action {

/**
 * @brief The type of action being commanded
 * 
 */
enum ActionType {
    GO_TO_POINT,
    GO_TO_POSE,
    SHOOT,
    COLLECT,
    PASS,
    DRIBBLE,
    CLEAR,
    MARK_ROBOT,
    ROTATE_TO_HEADING,
    UNKNOWN
};

class Action {
public:
    using Msg = rj_control_msgs::msg::Action;

    /**
     * @brief Construct a new unknown action
     * 
     */
    Action()
        : type_(ActionType::UNKNOWN) {}

    /**
     * @brief Construct a new Go To Point Action
     * 
     * @param go_to_point 
     */
    Action(GoToPoint go_to_point)
        : type_(ActionType::GO_TO_POINT), go_to_point_(go_to_point) {}

    /**
     * @brief Create a go to point Action
     * 
     * @param target 
     * @param avoid_ball 
     * @return Action 
     */
    static Action create_go_to_point(
        rj_geometry::Point target,
        bool avoid_ball,
        double tolerance = 0.05
    ) {
        return { GoToPoint(target, avoid_ball, tolerance) };
    }

    /**
     * @brief Construct a new Go To Pose Action
     * 
     * @param go_to_pose 
     */
    Action(GoToPose go_to_pose)
        : type_(ActionType::GO_TO_POSE), go_to_pose_(go_to_pose) {}

    /**
     * @brief Create a go to pose action
     * 
     * @param target 
     * @param avoid_ball 
     * @return Action 
     */
    static Action create_go_to_pose(
        rj_geometry::Pose target,
        bool avoid_ball,
        double position_tolerance = 0.05,
        double angular_tolerance = M_PI / 16
    ) {
        return { GoToPose(target, avoid_ball, position_tolerance, angular_tolerance) };
    }

    /**
     * @brief Construct a new shoot action
     * 
     * @param shoot 
     */
    Action(Shoot shoot)
        : type_(ActionType::SHOOT), shoot_(shoot) {}
    
    /**
     * @brief Create a shoot action
     * 
     * @param goal_location 
     * @param power 
     * @return Action 
     */
    static Action create_shoot(double goal_location, double power) {
        return { Shoot(goal_location, power) };
    }

    /**
     * @brief Construct a new collect action
     * 
     * @param collect 
     */
    Action(Collect collect)
        : type_(ActionType::COLLECT), collect_(collect) {}

    /**
     * @brief Create a collect action
     * 
     * @return Action 
     */
    static Action create_collect() {
        return { Collect() };
    }

    /**
     * @brief Construct a new pass action
     * 
     * @param pass 
     */
    Action(Pass pass)
        : type_(ActionType::PASS), pass_(pass) {}

    /**
     * @brief Create a pass action
     * 
     * @param robot_id 
     * @param power 
     * @return Action 
     */
    static Action create_pass(int robot_id, double power) {
        return { Pass(robot_id, power) };
    }

    /**
     * @brief Construct a new dribble action
     * 
     * @param dribble 
     */
    Action(Dribble dribble)
        : type_(ActionType::DRIBBLE), dribble_(dribble) {}

    /**
     * @brief Create a dribble action
     * 
     * @param target 
     * @return Action 
     */
    static Action create_dribble(rj_geometry::Pose target) {
        return { Dribble(target) };
    }

    /**
     * @brief Construct a new clear action
     * 
     * @param clear 
     */
    Action(Clear clear)
        : type_(ActionType::CLEAR), clear_(clear) {}

    /**
     * @brief Create a clear action
     * 
     * @param clear_target 
     * @param power 
     * @return Action 
     */
    static Action create_clear(rj_geometry::Point clear_target, double power) {
        return { Clear(clear_target, power) };
    }

    /**
     * @brief Construct a new mark robot action
     * 
     * @param mark_robot 
     */
    Action(MarkRobot mark_robot)
        : type_(ActionType::MARK_ROBOT), mark_robot_(mark_robot) {}

    /**
     * @brief Create a mark robot action
     * 
     * @param their_robot_id 
     * @param distance 
     * @return Action 
     */
    static Action create_mark_robot(int their_robot_id, double distance) {
        return { MarkRobot(their_robot_id, distance) };
    }

    /**
     * @brief Construct a new rotate to heading action
     * 
     * @param rotate_to_heading 
     */
    Action(RotateToHeading rotate_to_heading)
        : type_(ActionType::ROTATE_TO_HEADING), rotate_to_heading_(rotate_to_heading) {}

    /**
     * @brief Create a rotate to heading action
     * 
     * @param heading 
     * @param tolerance 
     * @return Action 
     */
    static Action create_rotate_to_heading(double heading, double tolerance) {
        return { RotateToHeading(heading, tolerance) };
    }

    /**
     * @brief Get the type of the action
     * 
     * @return ActionType 
     */
    [[nodiscard]] ActionType get_type() const {
        return type_;
    }

    /**
     * @brief Get the go to point action contained in this action
     * 
     * @return std::optional<GoToPoint> 
     */
    [[nodiscard]] std::optional<GoToPoint> go_to_point() const {
        return go_to_point_;
    }

    /**
     * @brief Get the go to pose action contained in this action
     * 
     * @return std::optional<GoToPose> 
     */
    [[nodiscard]] std::optional<GoToPose> go_to_pose() const {
        return go_to_pose_;
    }

    /**
     * @brief Get the shoot action contained in this action
     * 
     * @return std::optional<Shoot> 
     */
    [[nodiscard]] std::optional<Shoot> shoot() const {
        return shoot_;
    }

    /**
     * @brief Get the collect action contained in this action
     * 
     * @return std::optional<Collect> 
     */
    [[nodiscard]] std::optional<Collect> collect() const {
        return collect_;
    }

    /**
     * @brief Get the pass action contained in this action
     * 
     * @return std::optional<Pass> 
     */
    [[nodiscard]] std::optional<Pass> pass() const {
        return pass_;
    }

    /**
     * @brief Get the dribble action contained in this action
     * 
     * @return std::optional<Dribble> 
     */
    [[nodiscard]] std::optional<Dribble> dribble() const {
        return dribble_;
    }

    /**
     * @brief Get the clear action contained in this action
     * 
     * @return std::optional<Clear> 
     */
    [[nodiscard]] std::optional<Clear> clear() const {
        return clear_;
    }

    /**
     * @brief Get the mark robot action contained in this action
     * 
     * @return std::optional<MarkRobot> 
     */
    [[nodiscard]] std::optional<MarkRobot> mark_robot() const {
        return mark_robot_;
    }

    /**
     * @brief Get the rotate to heading action contained in this action
     * 
     * @return std::optional<RotateToHeading> 
     */
    [[nodiscard]] std::optional<RotateToHeading> rotate_to_heading() const {
        return rotate_to_heading_;
    }

private:
    // The type of action this action message contains
    ActionType type_;

    // The go to point action contained in the message
    std::optional<GoToPoint> go_to_point_;
    // The go to pose action contained in the message
    std::optional<GoToPose> go_to_pose_;
    // The shoot action contained in the message
    std::optional<Shoot> shoot_;
    // The collect action contained in the message
    std::optional<Collect> collect_;
    // The pass action contained in the message
    std::optional<Pass> pass_;
    // The dribble action contained in the message
    std::optional<Dribble> dribble_;
    // The clear action contained in the message
    std::optional<Clear> clear_;
    // The mark action contained in the message
    std::optional<MarkRobot> mark_robot_;
    // The rotate to heading action contained in the message
    std::optional<RotateToHeading> rotate_to_heading_;
};

} // namespace action

namespace rj_convert {

template <>
struct RosConverter<action::Action, action::Action::Msg> {
    static rj_control_msgs::msg::Action to_ros(const action::Action& from) {
        rj_control_msgs::msg::Action msg;
        switch (from.get_type()) {
            case action::ActionType::GO_TO_POINT:
                msg.action = rj_control_msgs::msg::Action::GO_TO_POINT;
                msg.go_to_point.push_back(convert_to_ros(from.go_to_point().value()));
                break;
            case action::ActionType::GO_TO_POSE:
                msg.action = rj_control_msgs::msg::Action::GO_TO_POSE;
                msg.go_to_pose.push_back(convert_to_ros(from.go_to_pose().value()));
                break;
            case action::ActionType::SHOOT:
                msg.action = rj_control_msgs::msg::Action::SHOOT;
                msg.shoot.push_back(convert_to_ros(from.shoot().value()));
                break;
            case action::ActionType::COLLECT:
                msg.action = rj_control_msgs::msg::Action::COLLECT;
                msg.collect.push_back(convert_to_ros(from.collect().value()));
                break;
            case action::ActionType::PASS:
                msg.action = rj_control_msgs::msg::Action::PASS;
                msg.pass_ball.push_back(convert_to_ros(from.pass().value()));
                break;
            case action::ActionType::DRIBBLE:
                msg.action = rj_control_msgs::msg::Action::DRIBBLE;
                msg.dribble.push_back(convert_to_ros(from.dribble().value()));
                break;
            case action::ActionType::CLEAR:
                msg.action = rj_control_msgs::msg::Action::CLEAR;
                msg.clear.push_back(convert_to_ros(from.clear().value()));
                break;
            case action::ActionType::MARK_ROBOT:
                msg.action = rj_control_msgs::msg::Action::MARK_ROBOT;
                msg.mark_robot.push_back(convert_to_ros(from.mark_robot().value()));
                break;
            case action::ActionType::ROTATE_TO_HEADING:
                msg.action = rj_control_msgs::msg::Action::ROTATE_TO_HEADING;
                msg.rotate_to_heading.push_back(convert_to_ros(from.rotate_to_heading().value()));
                break;
            case action::ActionType::UNKNOWN:
                msg.action = rj_control_msgs::msg::Action::UNKNOWN;
                break;
        }
        return msg;
    }

    static action::Action from_ros(const rj_control_msgs::msg::Action& from) {
        switch (from.action) {
            case rj_control_msgs::msg::Action::GO_TO_POINT:
                return {
                    action::GoToPoint(convert_from_ros(from.go_to_point[0]))
                };
            case rj_control_msgs::msg::Action::GO_TO_POSE:
                return {
                    action::GoToPose(convert_from_ros(from.go_to_pose[0]))
                };
            case rj_control_msgs::msg::Action::SHOOT:
                return {
                    action::Shoot(convert_from_ros(from.shoot[0]))
                };
            case rj_control_msgs::msg::Action::COLLECT:
                return {
                    action::Collect(convert_from_ros(from.collect[0]))
                };
            case rj_control_msgs::msg::Action::PASS:
                return {
                    action::Pass(convert_from_ros(from.pass_ball[0]))
                };
            case rj_control_msgs::msg::Action::DRIBBLE:
                return {
                    action::Dribble(convert_from_ros(from.dribble[0]))
                };
            case rj_control_msgs::msg::Action::CLEAR:
                return {
                    action::Clear(convert_from_ros(from.clear[0]))
                };
            case rj_control_msgs::msg::Action::MARK_ROBOT:
                return {
                    action::MarkRobot(convert_from_ros(from.mark_robot[0]))
                };
            case rj_control_msgs::msg::Action::ROTATE_TO_HEADING:
                return {
                    action::RotateToHeading(convert_from_ros(from.rotate_to_heading[0]))
                };
            case rj_control_msgs::msg::Action::UNKNOWN:
                return {

                };
        }

        return {};
    }
};

ASSOCIATE_CPP_ROS(action::Action, action::Action::Msg);

} // namespace rj_convert