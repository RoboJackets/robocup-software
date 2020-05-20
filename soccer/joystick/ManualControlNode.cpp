#include "ManualControlNode.hpp"

namespace joystick {
ManualControlNode::ManualControlNode(Context* context) : context_{context} {
}

void ManualControlNode::applyControlsToRobots(std::vector<OurRobot*>* robots) {
    // If there aren't any gamepads that are connected
    // or we don't have any robot assigned to control, return.
    if (gamepad_stack_.empty() || !manual_id_.has_value()) {
        return;
    }
    const int manual_id = manual_id_.value();

    // Find the robot that we are supposed to control
    const auto pred = [manual_id](const OurRobot* r) {
        return r->shell() == manual_id;
    };
    const auto it = std::find_if(robots->begin(), robots->end(), pred);

    // Return if we can't find the robot we're supposed to control
    if (it == robots->end()) {
        return;
    }

    int shell = (*it)->shell();

    // Modify robot->setpoint and robot->intent
    RobotIntent& intent = context_->robot_intents[shell];
    MotionSetpoint& setpoint = context_->motion_setpoints[shell];

    // translation
    setpoint.xvelocity = manual_controls_.x_vel;
    setpoint.yvelocity = manual_controls_.y_vel;

    // rotation
    setpoint.avelocity = manual_controls_.a_vel;

    // kick/chip
    bool kick = manual_controls_.kick || manual_controls_.chip;
    intent.trigger_mode =
        (kick ? (context_->game_settings.joystick_config.useKickOnBreakBeam
                     ? RobotIntent::TriggerMode::ON_BREAK_BEAM
                     : RobotIntent::TriggerMode::IMMEDIATE)
              : RobotIntent::TriggerMode::STAND_DOWN);
    intent.kcstrength = manual_controls_.kick_power;
    intent.shoot_mode = (manual_controls_.kick ? RobotIntent::ShootMode::KICK
                                               : RobotIntent::ShootMode::CHIP);

    // dribbler
    intent.dvelocity =
        manual_controls_.dribble ? manual_controls_.dribbler_power : 0;
}

GamepadCallbackFn ManualControlNode::getCallback() {
    return [this](const GamepadMessage& msg) { callback(msg); };
}

GamepadConnectedFn ManualControlNode::getOnConnect() {
    return [this](int unique_id) { onJoystickConnected(unique_id); };
}

GamepadDisconnectedFn ManualControlNode::getOnDisconnect() {
    return [this](int unique_id) { onJoystickDisconnected(unique_id); };
}

void ManualControlNode::callback(const GamepadMessage& msg) {
    if (msg.buttons.a) {
        std::cout << "A pressed!" << std::endl;
    }
}

void ManualControlNode::onJoystickConnected(int unique_id) {
    gamepad_stack_.emplace_back(unique_id);
}

void ManualControlNode::onJoystickDisconnected(int unique_id) {
    const auto is_instance = [unique_id](int other) -> bool {
        return unique_id == other;
    };

    // Remove it from the stack
    gamepad_stack_.erase(std::remove_if(gamepad_stack_.begin(),
                                        gamepad_stack_.end(), is_instance),
                         gamepad_stack_.end());
}

}  // namespace joystick
