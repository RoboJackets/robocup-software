#include "rj_joystick/manual_control_node.hpp"

#include <rj_utils/ros_params.hpp>

namespace joystick {

ManualControlNode::ManualControlNode() : rclcpp::Node("manual_control") {
    using rj_utils::declare_required_param;
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

    auto on_connect = [this](ManualController* controller) {
        controllers_.insert({controller, std::nullopt});
    };
    auto on_disconnect = [this](ManualController* controller) { remove_controller(controller); };

    providers_.push_back(
        std::make_unique<SDLControllerProvider>(true, params_, on_connect, on_disconnect));

    using rj_msgs::srv::ListJoysticks;
    list_joysticks_ = create_service<ListJoysticks>(
        "list_joysticks",
        [this]([[maybe_unused]] const ListJoysticks::Request::SharedPtr request,  // NOLINT
               ListJoysticks::Response::SharedPtr response) {                     // NOLINT
            for (const auto& [controller, robot] : controllers_) {
                response->descriptions.push_back(controller->get_description());
                response->uuids.push_back(controller->get_uuid());
                response->robots.push_back(robot.value_or(-1));
            }
        });

    using rj_msgs::srv::SetManual;
    set_manual_ = create_service<SetManual>(
        "select_manual",
        [this](const SetManual::Request::SharedPtr request,                 // NOLINT
               [[maybe_unused]] SetManual::Response::SharedPtr response) {  // NOLINT
            set_manual(request->controller_uuid,
                       request->connect ? std::make_optional(request->robot_id) : std::nullopt);
        });

    for (unsigned int i = 0; i < kNumShells; i++) {
        motion_setpoint_pubs_.push_back(create_publisher<rj_msgs::msg::MotionSetpoint>(
            control::topics::motion_setpoint_topic(i), 10));
        manipulator_setpoint_pubs_.push_back(create_publisher<rj_msgs::msg::ManipulatorSetpoint>(
            control::topics::manipulator_setpoint_topic(i), 10));
    }

    timer_ = create_wall_timer(std::chrono::microseconds(1'000'000 / 60), [this]() {
        for (const auto& provider : providers_) {
            provider->update();
        }

        for (const auto& [controller, robot] : controllers_) {
            if (robot.has_value()) {
                publish(robot.value(), controller->get_command());
            }
        }
    });
}

void ManualControlNode::set_manual(const std::string& uuid, std::optional<int> robot_id) {
    auto it = std::find_if(
        controllers_.begin(), controllers_.end(),
        [uuid](const auto& controller_pair) { return controller_pair.first->get_uuid() == uuid; });
    if (it == controllers_.end()) {
        SPDLOG_WARN("Requested invalid controller \"{}\"!", uuid);
        return;
    }

    auto& [controller, robot] = *it;
    if (robot.has_value()) {
        stop_robot(robot.value());
    }

    robot = robot_id;

    SPDLOG_INFO("Successfully connected controller \"{}\" to robot {}.", controller->get_uuid(),
                robot_id.value());
}

void ManualControlNode::remove_controller(ManualController* controller) {
    auto found = controllers_.find(controller);
    if (found != controllers_.end()) {
        controllers_.erase(found);
    } else {
        SPDLOG_WARN("Trying to remove non-existent controller!");
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
