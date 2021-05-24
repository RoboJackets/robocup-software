#include "manual_control_node.hpp"

#include "context.hpp"
#include "robot.hpp"

namespace joystick {

DEFINE_BOOL(kJoystickModule, use_field_oriented_drive, false, "Whether to use field oriented drive")
DEFINE_BOOL(kJoystickModule, kick_on_break_beam, false,
            "Wait for break beam when kick button is held")
DEFINE_BOOL(kJoystickModule, damped_translation, false, "Move slowly")
DEFINE_BOOL(kJoystickModule, damped_rotation, false, "Turn slowly")
DEFINE_INT64(kJoystickModule, manual_robot_id, -1, "Which robot to control manually (temporary)")
DEFINE_FLOAT64(kJoystickModule, max_rotation_speed, 4.0, "Maximum rotation speed, rad/s")
DEFINE_FLOAT64(kJoystickModule, max_damped_rotation_speed, 1.0,
               "Maximum damped rotation speed, rad/s")
DEFINE_FLOAT64(kJoystickModule, max_translation_speed, 2.0, "Maximum translation speed, m/s")
DEFINE_FLOAT64(kJoystickModule, max_damped_translation_speed, 0.5,
               "Maximum damped translation speed, m/s")

ManualControlNode::ManualControlNode(Context* context) : context_{context} {}

void ManualControlNode::run() {
    // Update list of connected gamepads
    update_gamepad_list();

    // "Manually" call the fake callback on the data in context_
    for (const auto& msg : context_->gamepad_messages) {
        callback(msg);
    }

    const auto robots = context_->state.self;
    // Set all robots to not be joystick controlled
    for (OurRobot* robot : robots) {
        robot->set_joystick_controlled(false);
    }

    // If there aren't any gamepads that are connected, return
    if (gamepad_stack_.empty()) {
        return;
    }

    const int manual_id = PARAM_manual_robot_id;

    // Find the robot that we are supposed to control
    const auto pred = [manual_id](const OurRobot* r) { return r->shell() == manual_id; };
    const auto it = std::find_if(robots.begin(), robots.end(), pred);

    // Return if we can't find the robot we're supposed to control
    if (it == robots.end()) {
        return;
    }

    OurRobot* robot = *it;

    robot->set_joystick_controlled(true);

    update_intent_and_setpoint(robot);
}

void ManualControlNode::update_intent_and_setpoint(OurRobot* robot) {
    int shell = robot->shell();

    // Modify robot->setpoint and robot->intent
    RobotIntent& intent = context_->robot_intents[shell];
    MotionSetpoint& setpoint = context_->motion_setpoints[shell];

    intent.is_active = true;

    float x_vel = controls_.x_vel;
    float y_vel = controls_.y_vel;

    // Field oriented control
    if (PARAM_use_field_oriented_drive) {
        // If robot isn't visible, then stop
        if (!robot->visible()) {
            intent = {};
            setpoint = {};
            return;
        }

        // Rotate x_vel and y_vel so that it's the field's x and y
        rj_geometry::Point translation{x_vel, y_vel};
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
    intent.trigger_mode = kick ? (PARAM_kick_on_break_beam ? RobotIntent::TriggerMode::ON_BREAK_BEAM
                                                           : RobotIntent::TriggerMode::IMMEDIATE)
                               : RobotIntent::TriggerMode::STAND_DOWN;

    // TODO(#1583): The controller should directly work in the (0, max_kick_speed] range
    constexpr float kMaxKickSpeed = 7.0;
    intent.kick_speed = static_cast<float>(controls_.kick_power) / kMaxKick * kMaxKickSpeed;
    intent.shoot_mode =
        controls_.kick ? RobotIntent::ShootMode::KICK : RobotIntent::ShootMode::CHIP;

    // dribbler
    intent.dribbler_speed = controls_.dribble ? controls_.dribbler_power : 0;
}

void ManualControlNode::update_gamepad_list() {
    gamepad_stack_ = context_->gamepads;
    update_joystick_valid();
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
    if (msg.unique_id != gamepad_stack_.front()) {
        return;
    }

    const auto now = RJ::now();
    // Dribbler Power
    if (msg.buttons.a) {
        if ((now - last_dribbler_time_) >= kDribbleStepTime) {
            controls_.dribbler_power = std::max(controls_.dribbler_power - 0.1, 0.0);
            last_dribbler_time_ = now;
        }
    } else if (msg.buttons.y) {
        if ((now - last_dribbler_time_) >= kDribbleStepTime) {
            controls_.dribbler_power = std::min(controls_.dribbler_power + 0.1, 1.0);
            last_dribbler_time_ = now;
        }
    } else {
        // Let dribbler speed change immediately
        last_dribbler_time_ = now - kDribbleStepTime;
    }

    // Kicker Power
    if (msg.buttons.x) {
        if ((now - last_kicker_time_) >= kKickerStepTime) {
            controls_.kick_power = std::max(controls_.kick_power - 0.1, 0.0);
            last_kicker_time_ = now;
        }
    } else if (msg.buttons.b) {
        if ((now - last_kicker_time_) >= kKickerStepTime) {
            controls_.kick_power = std::min(controls_.kick_power + 0.1, 1.0);
            last_kicker_time_ = now;
        }
    } else {
        last_kicker_time_ = now - kKickerStepTime;
    }

    // Kick True/False
    controls_.kick = msg.buttons.right_shoulder;

    // Chip
    controls_.chip = static_cast<float>(msg.triggers.right) / kAxisMax > kTriggerCutoff;

    // Rotation Velocity
    controls_.a_vel = -1.f * static_cast<float>(msg.sticks.right.x) / kAxisMax;

    // Translation Velocity
    float left_x = static_cast<float>(msg.sticks.left.x) / kAxisMax;
    float left_y = static_cast<float>(msg.sticks.left.y) / kAxisMax;

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

    apply_control_modifiers();
}

void ManualControlNode::apply_control_modifiers() {
    rj_geometry::Point trans{controls_.x_vel, controls_.y_vel};
    trans.clamp(std::sqrt(2.0));
    controls_.a_vel = std::clamp(controls_.a_vel, -1.f, 1.f);

    if (PARAM_damped_translation) {
        trans *= PARAM_max_damped_translation_speed;
    } else {
        trans *= PARAM_max_translation_speed;
    }

    if (PARAM_damped_rotation) {
        trans *= PARAM_max_damped_rotation_speed;
    } else {
        trans *= PARAM_max_rotation_speed;
    }

    // Scale up kicker and dribbler speeds
    controls_.dribbler_power *= kMaxDribble;
    controls_.kick_power *= kMaxKick;
}

void ManualControlNode::update_joystick_valid() const {
    context_->joystick_valid = !gamepad_stack_.empty();
}
}  // namespace joystick
