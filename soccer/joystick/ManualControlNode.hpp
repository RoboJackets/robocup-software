#pragma once

#include <Node.hpp>
#include <Robot.hpp>
#include <SystemState.hpp>
#include <functional>
#include <joystick/GamepadMessage.hpp>
#include <unordered_map>

namespace joystick {
static constexpr auto DribbleStepTime = RJ::Seconds(0.125);
static constexpr auto KickerStepTime = RJ::Seconds(0.125);

static constexpr float AXIS_MAX = 32768.0f;
static constexpr float TRIGGER_CUTOFF = 0.9;

/**
 * A node that receives joystick::GamepadMessage and converts that to
 * RobotIntent.
 *
 * For now (since the GUI has no way of assigning robots to multiple
 * joysticks) it maintains a stack of which gamepad connected first,
 * and uses the oldest connected gamepad.
 */
class ManualControlNode : Node {
public:
    ManualControlNode(Context* context);

    /**
     * Perform manual control things to our robots by
     * modifying intent and setpoint.
     */
    void run() override;

    static void createConfiguration(Configuration* cfg);

private:
    /**
     * Performs the logic for converting from a gamepad scheme
     * to controls for a robot.
     * @param msg
     */
    void callback(const GamepadMessage& msg);

    /**
     * Updates the current list of gamepads.
     */
    void updateGamepadList();

    /**
     * Updates the intent and setpoint using controls_;
     * @param robot
     */
    void updateIntentAndSetpoint(OurRobot* robot);

    /**
     * Updates the context_->joystick_valid
     */
    void updateJoystickValid() const;

    /**
     * Apply things like translation damping, rotation damping etc.
     */
    void applyControlModifiers();

    std::vector<int> gamepad_stack_;

    struct ManualControls {
        float x_vel;
        float y_vel;
        float a_vel;
        int kick_power;
        float dribbler_power;
        bool kick;
        bool dribble;
        bool chip;
    };

    Context* context_;

    ManualControls controls_{};

    static ConfigDouble* JoystickRotationMaxSpeed;
    static ConfigDouble* JoystickRotationMaxDampedSpeed;
    static ConfigDouble* JoystickTranslationMaxSpeed;
    static ConfigDouble* JoystickTranslationMaxDampedSpeed;

    // And then random state needed for the control logic
    RJ::Time last_dribbler_time_;
    RJ::Time last_kicker_time_;
};
}  // namespace joystick
