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

    inline void setManualID(int manual_id) { manual_id_ = manual_id; }

    GamepadCallbackFn getCallback();
    GamepadConnectedFn getOnConnect();
    GamepadDisconnectedFn getOnDisconnect();

private:
    void callback(const GamepadMessage& msg);
    void onJoystickConnected(int unique_id);
    void onJoystickDisconnected(int unique_id);

    std::optional<int> manual_id_;
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

    ManualControls manual_controls_;
    Context* context_;
};
}  // namespace joystick
