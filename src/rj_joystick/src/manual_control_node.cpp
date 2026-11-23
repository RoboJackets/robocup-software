#include "rj_joystick/manual_control_node.hpp"

#include <rj_common/game_constants.hpp>
#include <rj_common/ros_params.hpp>
#include <rj_common/topic_names.hpp>
#include <rj_common/utils.hpp>

#include "rj_joystick/input_map.hpp"

namespace joystick {

ManualControlNode::ManualControlNode()
    : rclcpp::Node("manual_control"),
      kick_power_(kNumShells, 0.5),
      dribble_power_(kNumShells, 0.0),
      commands_(kNumShells),
      previous_power_buttons_(kNumShells),
      has_command_(kNumShells, false) {
    ManualControlParams p{};
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
                if (name == "kick_power_increment") {
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

            if (updated.min_kick_speed < 0 || updated.max_kick_speed < 0) {
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
                command.translation.x() = -msg->axes.at(joystick::axis::kLeftX);
                command.translation.y() = msg->axes.at(joystick::axis::kLeftY);
                command.rotation = msg->axes.at(joystick::axis::kRightX);
                command.kick = msg->axes.at(joystick::axis::kTriggerRight) > kTriggerCutoff;
                command.chip = msg->buttons.at(joystick::button::kLeftShoulder) != 0;

                const bool kick_increase =
                    msg->buttons.at(joystick::button::kDpadRight) != 0;
                const bool kick_decrease =
                    msg->buttons.at(joystick::button::kDpadLeft) != 0;
                const bool dribble_increase =
                    msg->buttons.at(joystick::button::kDpadUp) != 0;
                const bool dribble_decrease =
                    msg->buttons.at(joystick::button::kDpadDown) != 0;
                auto& previous_buttons = previous_power_buttons_.at(i);

                if (kick_increase && !previous_buttons[0]) {
                    kick_power_.at(i) += params_->kick_power_increment;
                } else if (kick_decrease && !previous_buttons[1]) {
                    kick_power_.at(i) -= params_->kick_power_increment;
                }
                kick_power_.at(i) = std::clamp(kick_power_.at(i), 0.0, 1.0);

                if (dribble_increase && !previous_buttons[2]) {
                    dribble_power_.at(i) += params_->dribble_power_increment;
                } else if (dribble_decrease && !previous_buttons[3]) {
                    dribble_power_.at(i) -= params_->dribble_power_increment;
                }
                dribble_power_.at(i) = std::clamp(dribble_power_.at(i), 0.0, 1.0);
                previous_buttons = {kick_increase, kick_decrease, dribble_increase,
                                    dribble_decrease};

                if (command.kick || command.chip) {
                    command.kick_power = kick_power_.at(i);
                }

                command.dribble_power = dribble_power_.at(i);
                commands_.at(i) = command;
                has_command_.at(i) = true;
            }));
    }

    for (unsigned int i = 0; i < kNumShells; i++) {
        motion_setpoint_pubs_.push_back(create_publisher<rj_msgs::msg::MotionSetpoint>(
            control::topics::motion_setpoint_topic(i), 10));
        manipulator_setpoint_pubs_.push_back(create_publisher<rj_msgs::msg::ManipulatorSetpoint>(
            control::topics::manipulator_setpoint_topic(i), 10));
    }

    tick_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { tick(); });
}

void ManualControlNode::tick() {
    const auto now = std::chrono::steady_clock::now();
    for (size_t i = 0; i < kNumShells; ++i) {
        if (!has_command_.at(i)) {
            continue;
        }
        if (now - last_command_times_.at(i) > std::chrono::milliseconds(1000)) {
            stop_robot(static_cast<int>(i));
            has_command_.at(i) = false;
            continue;
        }
        publish(static_cast<int>(i), commands_.at(i));
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

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<joystick::ManualControlNode>());
    rclcpp::shutdown();
    return 0;
}
