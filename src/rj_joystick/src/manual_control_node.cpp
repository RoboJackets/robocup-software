#include "rj_joystick/manual_control_node.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace joystick {

ManualControlNode::ManualControlNode() : rclcpp::Node("manual_control") {
    declare_params();
    param_callback_handle_ = add_on_set_parameters_callback(
        [this](const auto& params) { return on_param_change(params); });

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

void ManualControlNode::declare_params() {
    auto declare = [this](const std::string& name, auto default_value, const std::string& desc) {
        rcl_interfaces::msg::ParameterDescriptor d;
        d.description = desc;
        declare_parameter(name, default_value, d);
    };

    declare("use_field_oriented_drive", params_.use_field_oriented_drive,
            "Whether to use field oriented drive");
    declare("kick_on_break_beam", false, "Wait for break beam when kick button is held");
    declare("damped_translation", params_.damped_translation, "Move slowly");
    declare("damped_rotation", params_.damped_rotation, "Turn slowly");
    declare("max_rotation_speed", params_.max_rotation_speed, "Maximum rotation speed, rad/s");
    declare("max_damped_rotation_speed", params_.max_damped_rotation_speed,
            "Maximum damped rotation speed, rad/s");
    declare("max_translation_speed", params_.max_translation_speed,
            "Maximum translation speed, m/s");
    declare("max_damped_translation_speed", params_.max_damped_translation_speed,
            "Maximum damped translation speed, m/s");
    declare("kick_power_increment", params_.kick_power_increment, "Kick power increment, 0-1");
    declare("dribble_power_increment", params_.dribble_power_increment,
            "Dribble power increment, 0-1");
    declare("min_kick_speed", 0.0, "Minimum kick speed, m/s");
    declare("max_kick_speed", 15.0, "Maximum kick speed, m/s");

    get_parameter("use_field_oriented_drive", params_.use_field_oriented_drive);
    get_parameter("damped_translation", params_.damped_translation);
    get_parameter("damped_rotation", params_.damped_rotation);
    get_parameter("max_rotation_speed", params_.max_rotation_speed);
    get_parameter("max_damped_rotation_speed", params_.max_damped_rotation_speed);
    get_parameter("max_translation_speed", params_.max_translation_speed);
    get_parameter("max_damped_translation_speed", params_.max_damped_translation_speed);
    get_parameter("kick_power_increment", params_.kick_power_increment);
    get_parameter("dribble_power_increment", params_.dribble_power_increment);
}

rcl_interfaces::msg::SetParametersResult ManualControlNode::on_param_change(
    const std::vector<rclcpp::Parameter>& parameters) {
    for (const auto& p : parameters) {
        if (p.get_name() == "use_field_oriented_drive")
            params_.use_field_oriented_drive = p.as_bool();
        else if (p.get_name() == "damped_translation")
            params_.damped_translation = p.as_bool();
        else if (p.get_name() == "damped_rotation")
            params_.damped_rotation = p.as_bool();
        else if (p.get_name() == "max_rotation_speed")
            params_.max_rotation_speed = p.as_double();
        else if (p.get_name() == "max_damped_rotation_speed")
            params_.max_damped_rotation_speed = p.as_double();
        else if (p.get_name() == "max_translation_speed")
            params_.max_translation_speed = p.as_double();
        else if (p.get_name() == "max_damped_translation_speed")
            params_.max_damped_translation_speed = p.as_double();
        else if (p.get_name() == "kick_power_increment")
            params_.kick_power_increment = p.as_double();
        else if (p.get_name() == "dribble_power_increment")
            params_.dribble_power_increment = p.as_double();
        // kick_on_break_beam deliberately excluded — nothing caches it.
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
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
    bool kick_on_break_beam = false;
    get_parameter("kick_on_break_beam", kick_on_break_beam);
    uint8_t trigger_mode = kick_on_break_beam
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

    double min_kick_speed = 0.0;
    double max_kick_speed = 15.0;
    get_parameter("min_kick_speed", min_kick_speed);
    get_parameter("max_kick_speed", max_kick_speed);

    double kick_speed = lerp(min_kick_speed, max_kick_speed, command.kick_power);
    manipulator_setpoint_pubs_.at(robot_id)->publish(
        rj_msgs::build<rj_msgs::msg::ManipulatorSetpoint>()
            .shoot_mode(shoot_mode)
            .trigger_mode(trigger_mode)
            .kick_speed(kick_speed)
            .dribbler_speed(static_cast<float>(command.dribble_power)));
}

}  // namespace joystick