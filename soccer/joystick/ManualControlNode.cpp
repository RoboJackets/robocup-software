#include "ManualControlNode.hpp"

namespace joystick {
ManualControlNode::ManualControlNode(Context* context) : context_{context} {}

void ManualControlNode::applyControlsToRobots(std::vector<OurRobot*>* robots) {
    // Set all robots to not be joystick controlled
    for (OurRobot* robot : *robots) {
        robot->setJoystickControlled(false);
    }

    // If there aren't any gamepads that are connected, return
    if (gamepad_stack_.empty()) {
        return;
    }

    const int manual_id = context_->game_settings.joystick_config.manualID;

    // Find the robot that we are supposed to control
    const auto pred = [manual_id](const OurRobot* r) {
        return r->shell() == manual_id;
    };
    const auto it = std::find_if(robots->begin(), robots->end(), pred);

    // Return if we can't find the robot we're supposed to control
    if (it == robots->end()) {
        return;
    }

    OurRobot* robot = *it;
    int shell = robot->shell();

    robot->setJoystickControlled(true);

    // Modify robot->setpoint and robot->intent
    RobotIntent& intent = context_->robot_intents[shell];
    MotionSetpoint& setpoint = context_->motion_setpoints[shell];

    intent.is_active = true;

    float x_vel = controls_.x_vel;
    float y_vel = controls_.y_vel;

    // Field oriented control
    if (robot->visible() &&
        context_->game_settings.joystick_config.useFieldOrientedDrive) {
        Geometry2d::Point translation{x_vel, y_vel};
        translation.rotate(-M_PI / 2 - robot->angle());

        x_vel = static_cast<float>(translation.x());
        y_vel = static_cast<float>(translation.y());
    }

    // translation
    setpoint.xvelocity = x_vel;
    setpoint.yvelocity = y_vel;

    // rotation
    setpoint.avelocity = controls_.a_vel;

    // kick/chip
    bool kick = controls_.kick || controls_.chip;
    intent.trigger_mode =
        kick ? (context_->game_settings.joystick_config.useKickOnBreakBeam
                    ? RobotIntent::TriggerMode::ON_BREAK_BEAM
                    : RobotIntent::TriggerMode::IMMEDIATE)
             : RobotIntent::TriggerMode::STAND_DOWN;
    intent.kcstrength = controls_.kick_power;
    intent.shoot_mode = controls_.kick ? RobotIntent::ShootMode::KICK
                                       : RobotIntent::ShootMode::CHIP;

    // dribbler
    intent.dvelocity = controls_.dribble ? controls_.dribbler_power : 0;
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
    // Scheme:
    //   Y              - Increase dribbler power
    //   A              - Decrease dribbler power
    //
    //   B              - Increase kicker power
    //   X              - Decrease kicker power
    //
    //   RightShoulder  - Kick
    //   TriggerRight   - Chip
    //
    //   RightStickX    - Rotation
    //   LeftStickX     - Translation X
    //   LeftStickY     - Translation Y
    //
    //   DPAD           - "Align along an axis using the DPAD"

    const auto now = RJ::now();
    // Dribbler Power
    if (msg.buttons.a) {
        if ((now - last_dribbler_time_) >= DribbleStepTime) {
            controls_.dribbler_power =
                std::max(controls_.dribbler_power - 0.1, 0.0);
            last_dribbler_time_ = now;
        }
    } else if (msg.buttons.y) {
        if ((now - last_dribbler_time_) >= DribbleStepTime) {
            controls_.dribbler_power =
                std::min(controls_.dribbler_power + 0.1, 1.0);
            last_dribbler_time_ = now;
        }
    } else {
        // Let dribbler speed change immediately
        last_dribbler_time_ = now - DribbleStepTime;
    }

    // Kicker Power
    if (msg.buttons.x) {
        if ((now - last_kicker_time_) >= KickerStepTime) {
            controls_.kick_power = std::max(controls_.kick_power - 0.1, 0.0);
            last_kicker_time_ = now;
        }
    } else if (msg.buttons.b) {
        if ((now - last_kicker_time_) >= KickerStepTime) {
            controls_.kick_power = std::min(controls_.kick_power + 0.1, 1.0);
            last_kicker_time_ = now;
        }
    } else {
        last_kicker_time_ = now - KickerStepTime;
    }

    // Kick True/False
    controls_.kick = msg.buttons.right_shoulder;

    // Chip
    controls_.chip = msg.triggers.right / AXIS_MAX > TRIGGER_CUTOFF;

    // Rotation Velocity
    controls_.a_vel = -1 * msg.sticks.right.x / AXIS_MAX;

    // Translation Velocity
    float left_x = static_cast<float>(msg.sticks.left.x) / AXIS_MAX;
    float left_y = static_cast<float>(msg.sticks.left.y) / AXIS_MAX;

    // Align along an axis using the DPAD as modifier buttons
    float trans_x = left_x;
    float trans_y = left_y;
    if (msg.dpad.down) {
        trans_y = -std::abs(left_y);
        trans_x = 0;
    } else if (msg.dpad.up) {
        trans_y = std::abs(left_y);
        trans_x = 0;
    } else if (msg.dpad.left) {
        trans_y = 0;
        trans_x = -std::abs(left_x);
    } else if (msg.dpad.right) {
        trans_y = 0;
        trans_x = std::abs(left_x);
    }

    // Floating point precision error rounding
    if (controls_.kick_power < 1e-1) {
        controls_.kick_power = 0;
    }
    if (controls_.dribbler_power < 1e-1) {
        controls_.dribbler_power = 0;
    }
    if (std::abs(controls_.a_vel) < 5e-2) {
        controls_.a_vel = 0;
    }
    if (std::abs(trans_x) < 5e-2) {
        trans_x = 0;
    }
    if (std::abs(trans_y) < 5e-2) {
        trans_y = 0;
    }

    controls_.x_vel = trans_x;
    controls_.y_vel = trans_y;
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
