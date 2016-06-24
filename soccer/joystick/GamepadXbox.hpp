#pragma once

#include "Joystick.hpp"

#include "gamepad.h"

/**
 * @brief Logitecch Gamepad/Joystick used to control robots
 */
class GamepadXbox : public Joystick {
public:
    GamepadXbox();
    ~GamepadXbox();

    void reset() override;
    void update() override;
    JoystickControlValues getJoystickControlValues() override;

    bool valid() const override;

private:
    GAMEPAD_DEVICE _id;

    JoystickControlValues _controls;

    RJ::Time _lastDribblerTime;
    RJ::Time _lastKickerTime;
};
