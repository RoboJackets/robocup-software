#include "manual_control_node.hpp"

#include "context.hpp"

namespace joystick {

DEFINE_BOOL(kJoystickModule, use_field_oriented_drive, false, "Whether to use field oriented drive")
DEFINE_BOOL(kJoystickModule, kick_on_break_beam, false,
            "Wait for break beam when kick button is held")
DEFINE_BOOL(kJoystickModule, damped_translation, false, "Move slowly")
DEFINE_BOOL(kJoystickModule, damped_rotation, false, "Turn slowly")
DEFINE_FLOAT64(kJoystickModule, max_rotation_speed, 4.0, "Maximum rotation speed, rad/s")
DEFINE_FLOAT64(kJoystickModule, max_damped_rotation_speed, 1.0,
               "Maximum damped rotation speed, rad/s")
DEFINE_FLOAT64(kJoystickModule, max_translation_speed, 2.0, "Maximum translation speed, m/s")
DEFINE_FLOAT64(kJoystickModule, max_damped_translation_speed, 0.5,
               "Maximum damped translation speed, m/s")
DEFINE_FLOAT64(kJoystickModule, kick_power_increment, 0.1, "Kick power increment, 0-1")
DEFINE_FLOAT64(kJoystickModule, dribble_power_increment, 0.1, "Dribble power increment, 0-1")

ManualControlNode::ManualControlNode()
    : rclcpp::Node("manual_control"), param_provider_(this, kJoystickModule) {
    auto on_connect = [this](ManualController* controller) {
        controllers_.insert({controller, std::nullopt});
    };
    auto on_disconnect = [this](ManualController* controller) { remove_controller(controller); };

    providers_.push_back(std::make_unique<SDLControllerProvider>(true, on_connect, on_disconnect));

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

    for (int i = 0; i < kNumShells; i++) {
        motion_setpoint_pubs_.push_back(create_publisher<rj_msgs::msg::MotionSetpoint>(
            control::topics::motion_setpoint_pub(i), 10));
        manipulator_setpoint_pubs_.push_back(create_publisher<rj_msgs::msg::ManipulatorSetpoint>(
            control::topics::manipulator_setpoint_pub(i), 10));
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
        SPDLOG_WARN("Requested invalid controller {}", uuid);
        return;
    }

    auto& [controller, robot] = *it;
    if (robot.has_value()) {
        stop_robot(robot.value());
    }

    robot = robot_id;
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
    uint8_t trigger_mode = PARAM_kick_on_break_beam
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

    manipulator_setpoint_pubs_.at(robot_id)->publish(
        rj_msgs::build<rj_msgs::msg::ManipulatorSetpoint>()
            .shoot_mode(shoot_mode)
            .trigger_mode(trigger_mode)
            .kick_strength(static_cast<int8_t>(command.kick_power * 255))
            .dribbler_speed(static_cast<float>(command.dribble_power)));
}

}  // namespace joystick
