#pragma once

#include "Joystick.hpp"

#include <SDL.h>

/**
 * @brief Logitecch Gamepad/Joystick used to control robots
 */
class GamepadController : public Joystick {
public:
    GamepadController();
    ~GamepadController();

    void reset() override;
    void update() override;
    JoystickControlValues getJoystickControlValues() override;

    bool valid() const override;

private:
    SDL_GameController* _controller;

    JoystickControlValues _controls;

    RJ::Time _lastDribblerTime;
    RJ::Time _lastKickerTime;

    void openJoystick();
    void closeJoystick();
    bool connected;
};
