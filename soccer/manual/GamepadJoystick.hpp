#pragma once

#include "InputDevice.hpp"

#include <SDL.h>

/**
 * @brief Logitecch Gamepad/Joystick used to control robots
 *
 * Used to control the Logitech F310 and XBOX controllers
 *
 * This has been replaced by GamepadController
 */
class GamepadJoystick : public InputDevice {
public:
    GamepadJoystick();
    ~GamepadJoystick();


    void reset() override;
    void update() override;
    InputDeviceControlValues getInputDeviceControlValues() override;

    bool valid() const override;

private:
    SDL_Joystick* _joystick;

    InputDeviceControlValues _controls;

    RJ::Time _lastDribblerTime;
    RJ::Time _lastKickerTime;
};
