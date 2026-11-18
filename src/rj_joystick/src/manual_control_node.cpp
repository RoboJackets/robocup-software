#include "rj_joystick/manual_control_node.hpp"

#include <rj_common/ros_params.hpp>

namespace joystick {

ManualControlNode::ManualControlNode() : rclcpp::Node("manual_control") {
    ManualControlParams p{};
    p.max_rotation_speed = declare_required_param<double>(*this, "max_rotation_speed",
                                                          "Maximum rotation speed, rad/s");
    p.max_damped_rotation_speed = declare_required_param<double>(
        *this, "max_damped_rotation_speed", "Maximum rotation speed while holding shift, rad/s");
    p.max_translation_speed = declare_required_param<double>(*this, "max_translation_speed",
                                                             "Maximum translation speed, m/s");
    p.max_damped_translation_speed =
        declare_required_param<double>(*this, "max_damped_translation_speed",
                                       "Maximum translation speed while holding shift, m/s");
    p.kick_power_increment = declare_required_param<double>(
        *this, "kick_power_increment", "Kick power change per button press, 0-1");
    p.dribble_power_increment = declare_required_param<double>(
        *this, "dribble_power_increment", "Dribble power change per button press, 0-1");
    p.kick_on_break_beam = declare_required_param<bool>(
        *this, "kick_on_break_beam", "Wait for break beam when kick button is held");
    p.min_kick_speed = declare_required_param<double>(*this, "min_kick_speed",
                                                      "Kick speed at zero kick power, m/s");
    p.max_kick_speed = declare_required_param<double>(*this, "max_kick_speed",
                                                      "Kick speed at full kick power, m/s");
    params_ = std::make_shared<ManualControlParams>(p);
    param_callback_handle_ =
        add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;

            ManualControlParams updated = *params_;
            for (const auto& param : params) {
                const auto& name = param.get_name();
                if (name == "max_rotation_speed") {
                    updated.max_rotation_speed = param.as_double();
                } else if (name == "max_damped_rotation_speed") {
                    updated.max_damped_rotation_speed = param.as_double();
                } else if (name == "max_translation_speed") {
                    updated.max_translation_speed = param.as_double();
                } else if (name == "max_damped_translation_speed") {
                    updated.max_damped_translation_speed = param.as_double();
                } else if (name == "kick_power_increment") {
                    updated.kick_power_increment = param.as_double();
                } else if (name == "dribble_power_increment") {
                    updated.dribble_power_increment = param.as_double();
                } else if (name == "kick_on_break_beam") {
                    updated.kick_on_break_beam = param.as_bool();
                } else if (name == "min_kick_speed") {
                    updated.min_kick_speed = param.as_double();
                } else if (name == "max_kick_speed") {
                    updated.max_kick_speed = param.as_double();
                }
            }

            if (updated.max_rotation_speed < 0 || updated.max_damped_rotation_speed < 0 ||
                updated.max_translation_speed < 0 || updated.max_damped_translation_speed < 0 ||
                updated.min_kick_speed < 0 || updated.max_kick_speed < 0) {
                result.successful = false;
                result.reason = "manual control speeds must be nonnegative";
                return result;
            }
            if (updated.kick_power_increment < 0 || updated.kick_power_increment > 1 ||
                updated.dribble_power_increment < 0 || updated.dribble_power_increment > 1) {
                result.successful = false;
                result.reason = "power increments must be between 0 and 1";
                return result;
            }

            *params_ = updated;
            return result;
        });

    for (unsigned int i = 0; i < kNumShells; i++) {
        last_command_times_.push_back(std::chrono::steady_clock::now());
    }

    for (unsigned int i = 0; i < kNumShells; i++) {
        joy_subs_.push_back(create_subscription<sensor_msgs::msg::Joy>(
            joystick::topics::joystick_topic(i), 10,
            [this, i](const sensor_msgs::msg::Joy::SharedPtr msg) {
                last_command_times_.at(i) = std::chrono::steady_clock::now();
                ControllerCommand command;
                command.translation.x() = msg->axes.at(joystick::axis::kLeftY);
                command.translation.y() = -msg->axes.at(joystick::axis::kLeftX);
                command.rotation = -msg->axes.at(joystick::axis::kRightX);
                command.kick = msg->axes.at(joystick::axis::kTriggerRight) > kTriggerCutoff;
                command.chip = msg->buttons.at(joystick::button::kLeftShoulder) != 0;

                if (msg->buttons.at(joystick::button::kDpadRight) != 0) {
                    kick_power_.at(i) += params_->kick_power_increment;
                } else if (msg->buttons.at(joystick::button::kDpadLeft) != 0) {
                    kick_power_.at(i) -= params_->kick_power_increment;
                }
                kick_power_.at(i) = std::clamp(kick_power_.at(i), 0.0, 1.0);

                if (msg->buttons.at(joystick::button::kDpadUp) != 0) {
                    dribble_power_.at(i) += params_->dribble_power_increment;
                } else if (msg->buttons.at(joystick::button::kDpadDown) != 0) {
                    dribble_power_.at(i) -= params_->dribble_power_increment;
                }
                dribble_power_.at(i) = std::clamp(dribble_power_.at(i), 0.0, 1.0);

                if (command.kick || command.chip) {
                    command.kick_power = kick_power_.at(i);
                }

                command.dribble_power = dribble_power_.at(i);
                publish(i, command);
            }));
    }

    for (unsigned int i = 0; i < kNumShells; i++) {
        stop_robot_timers_.push_back(create_wall_timer(
            std::chrono::milliseconds(100),
            [this, i]() {
                auto now = std::chrono::steady_clock::now();
                auto last_command_time = last_command_times_.at(i);
                if (now - last_command_time > std::chrono::milliseconds(1000)) {
                    stop_robot(i);
                }
            }));
    }

    for (unsigned int i = 0; i < kNumShells; i++) {
        motion_setpoint_pubs_.push_back(create_publisher<rj_msgs::msg::MotionSetpoint>(
            control::topics::motion_setpoint_topic(i), 10));
        manipulator_setpoint_pubs_.push_back(create_publisher<rj_msgs::msg::ManipulatorSetpoint>(
            control::topics::manipulator_setpoint_topic(i), 10));
    }
}

void ManualControlNode::stop_robot(int robot_id) {
    motion_setpoint_pubs_.at(robot_id)->publish(rj_msgs::msg::MotionSetpoint{});
    manipulator_setpoint_pubs_.at(robot_id)->publish(rj_msgs::msg::ManipulatorSetpoint{});
}

void ManualControlNode::publish(int robot_id, const ControllerCommand& command) {
    motion_setpoint_pubs_.at(robot_id)->publish(rj_msgs::build<rj_msgs::msg::MotionSetpoint>()
                                                    .velocity_x_mps(command.translation.x())
                                                    .velocity_y_mps(command.translation.y())
                                                    .velocity_z_radps(command.rotation));
    uint8_t trigger_mode = params_->kick_on_break_beam
                               ? rj_msgs::msg::ManipulatorSetpoint::TRIGGER_MODE_ON_BREAK_BEAM
                               : rj_msgs::msg::ManipulatorSetpoint::TRIGGER_MODE_IMMEDIATE;
    uint8_t shoot_mode = rj_msgs::msg::ManipulatorSetpoint::SHOOT_MODE_CHIP;
    if (command.kick) {
        shoot_mode = rj_msgs::msg::ManipulatorSetpoint::SHOOT_MODE_KICK;
    } else if (command.chip) {
        shoot_mode = rj_msgs::msg::ManipulatorSetpoint::SHOOT_MODE_CHIP;
    } else {
        trigger_mode = rj_msgs::msg::ManipulatorSetpoint::TRIGGER_MODE_STAND_DOWN;
    }

    double kick_speed = lerp(params_->min_kick_speed, params_->max_kick_speed, command.kick_power);
    manipulator_setpoint_pubs_.at(robot_id)->publish(
        rj_msgs::build<rj_msgs::msg::ManipulatorSetpoint>()
            .shoot_mode(shoot_mode)
            .trigger_mode(trigger_mode)
            .kick_speed(kick_speed)
            .dribbler_speed(static_cast<float>(command.dribble_power)));
}

}  // namespace joystick
