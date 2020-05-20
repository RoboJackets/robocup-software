#pragma once

#include <Robot.hpp>
#include <SystemState.hpp>
#include <functional>
#include <joystick/GamepadMessage.hpp>
#include <unordered_map>

namespace joystick {
using GamepadCallbackFn = std::function<void(const GamepadMessage&)>;
using GamepadConnectedFn = std::function<void(int unique_id)>;
using GamepadDisconnectedFn = std::function<void(int unique_id)>;

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
class ManualControlNode {
public:
    ManualControlNode(Context* context);

    /**
     * Perform manual control things to the robots passed in by
     * modifying intent and setpoint.
     *
     * @param robots
     */
    void applyControlsToRobots(std::vector<OurRobot*>* robots);

    /**
     * @return std::function for the callback
     */
    GamepadCallbackFn getCallback();

    /**
     * @return std::function for onJoystickConnected
     */
    GamepadConnectedFn getOnConnect();

    /**
     * @return std::function for onJoystickDisconnected
     */
    GamepadDisconnectedFn getOnDisconnect();

    static void createConfiguration(Configuration* cfg);

private:
    void callback(const GamepadMessage& msg);
    void onJoystickConnected(int unique_id);
    void onJoystickDisconnected(int unique_id);

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
