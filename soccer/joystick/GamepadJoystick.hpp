#pragma once

#include "Joystick.hpp"

#include <SDL.h>

/**
 * @brief Logitecch Gamepad/Joystick used to control robots
 *
 * Used to control the Logitech F310 and XBOX controllers
 */
class GamepadJoystick : public Joystick {
public:
    GamepadJoystick();
    ~GamepadJoystick();

    void reset() override;
    void update() override;
    JoystickControlValues getJoystickControlValues() override;

    bool valid() const override;

private:
    SDL_Joystick* _joystick;

    JoystickControlValues _controls;

    RJ::Time _lastDribblerTime;
    RJ::Time _lastKickerTime;
};
