/**
 * @file control_command.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief A control command (linear motion + actuator) created by a controller and sent to
 * the radio to actuate on
 * @version 0.1
 * @date 2026-01-04
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <rj_convert/ros_convert.hpp>
#include <rj_control_msgs/msg/control_command.hpp>
#include <rj_geometry/pose.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_protos/ssl_simulation_robot_control.pb.h>

namespace control {

/**
 * @brief Which of the actuators should the kicker / chipper use
 * 
 */
enum ShootMode {
    KICK = rj_control_msgs::msg::ControlCommand::SHOOT_MODE_KICK,
    CHIP = rj_control_msgs::msg::ControlCommand::SHOOT_MODE_CHIP,
};

/**
 * @brief How should the kicker / chipper be triggered
 * 
 */
enum TriggerMode {
    STAND_DOWN = rj_control_msgs::msg::ControlCommand::TRIGGER_MODE_STAND_DOWN,
    IMMEDIATE = rj_control_msgs::msg::ControlCommand::TRIGGER_MODE_IMMEDIATE,
    ON_BREAK_BEAM = rj_control_msgs::msg::ControlCommand::TRIGGER_MODE_ON_BREAK_BEAM,
};

/**
 * @brief A full motion command (i.e. linear motion + actuators) for a robot
 * 
 */
class ControlCommand {
public:
    using Msg = rj_control_msgs::msg::ControlCommand;

    /**
     * @brief Construct a new Control Command
     * 
     */
    ControlCommand() = default;

    /**
     * @brief Construct a new Control Command
     * 
     * @param shoot_mode 
     * @param trigger_mode 
     * @param kick_strength 
     * @param dribble_speed 
     * @param velocity 
     */
    ControlCommand(
        ShootMode shoot_mode,
        TriggerMode trigger_mode,
        float kick_strength, //NOLINT(bugprone-easily-swappable-parameters)
        float dribble_speed,
        rj_geometry::Twist velocity
    ): shoot_mode_(shoot_mode),
       trigger_mode_(trigger_mode),
       kick_strength_(kick_strength),
       dribble_speed_(dribble_speed),
       velocity_(velocity) {}

    /**
     * @brief Construct a new Control Command with only a velocity componenet
     * 
     * @param velocity 
     */
    ControlCommand(rj_geometry::Twist velocity): velocity_(velocity) {}

    /**
     * @brief Convert the control command into an ssl-simulation command for
     * the robot
     * 
     * @param robot_id The shell id of the robot to command
     * @param command The robot command to copy the data into
     * @return RobotCommand 
     */
    void as_sim_command(int robot_id, RobotCommand* command) const;

    /**
     * @brief Get the shoot mode of the command
     * 
     * @return ShootMode 
     */
    [[nodiscard]] ShootMode shoot_mode() const {
        return shoot_mode_;
    }

    /**
     * @brief Set the shoot mode of the command
     * 
     * @param shoot_mode 
     */
    void set_shoot_mode(ShootMode shoot_mode) {
        shoot_mode_ = shoot_mode;
    }

    /**
     * @brief Get the trigger mode of the command
     * 
     * @return TriggerMode 
     */
    [[nodiscard]] TriggerMode trigger_mode() const {
        return trigger_mode_;
    }

    /**
     * @brief Set the trigger mode of the command
     * 
     * @param trigger_mode 
     */
    void set_trigger_mode(TriggerMode trigger_mode) {
        trigger_mode_ = trigger_mode;
    }

    /**
     * @brief Get the kick strength of the command
     * 
     * @return float 
     */
    [[nodiscard]] float kick_strength() const {
        return kick_strength_;
    }

    /**
     * @brief Set the kick strength of the command
     * 
     * @param kick_strength 
     */
    void set_kick_strength(float kick_strength) {
        kick_strength_ = kick_strength;
    }

    /**
     * @brief Get the dribble speed of the command
     * 
     * @return float 
     */
    [[nodiscard]] float dribble_speed() const {
        return dribble_speed_;
    }

    /**
     * @brief Set the dribble speed of the command
     * 
     * @param dribble_speed 
     */
    void set_dribble_speed(float dribble_speed) {
        dribble_speed_ = dribble_speed;
    }

    /**
     * @brief Get the velocity of the command
     * 
     * @return rj_geometry::Twist 
     */
    [[nodiscard]] rj_geometry::Twist velocity() const {
        return velocity_;
    }

    /**
     * @brief Set the velocity of the command
     * 
     * @param velocity 
     */
    void set_velocity(rj_geometry::Twist velocity) {
        velocity_ = velocity;
    }

private:
    // How should the robot kick the ball
    ShootMode shoot_mode_ = ShootMode::KICK;
    // How should the kicker be actuated
    TriggerMode trigger_mode_ = TriggerMode::STAND_DOWN;
    // How much power should be used to kick [0, 1]
    float kick_strength_ = 0.0;
    // How fast should the dribbler be moving ([-1, 1]) with 1 being towards us
    float dribble_speed_ = 0.0;
    // The (x, y, w) velocity the robot should move at
    rj_geometry::Twist velocity_ = {0.0, 0.0, 0.0};
};

} // namespace control

namespace rj_convert {

template <>
struct RosConverter<control::ControlCommand, control::ControlCommand::Msg> {
    static control::ControlCommand::Msg to_ros(const control::ControlCommand& from) {
        control::ControlCommand::Msg msg;
        msg.shoot_mode = from.shoot_mode();
        msg.trigger_mode = from.trigger_mode();
        msg.kick_strength = from.kick_strength();
        msg.dribble_speed = from.dribble_speed();
        msg.velocity = convert_to_ros(from.velocity());
        return msg;
    }

    static control::ControlCommand from_ros(const control::ControlCommand::Msg& from) {
        return {
            static_cast<control::ShootMode>(from.shoot_mode),
            static_cast<control::TriggerMode>(from.trigger_mode),
            from.kick_strength,
            from.dribble_speed,
            convert_from_ros(from.velocity)
        };
    }
};

ASSOCIATE_CPP_ROS(control::ControlCommand, control::ControlCommand::Msg);

} // namespace rj_convert