#pragma once

#include "InputDevice.hpp"

#include <SDL.h>

/**
 * @brief Logitecch Gamepad/Joystick used to control robots
 */
class GamepadController : public InputDevice {
public:
    GamepadController();
    ~GamepadController();

    static bool GamepadController::initDeviceType();

    void reset() override;
    void update() override;
    InputDeviceControlValues getInputDeviceControlValues() override;

    bool valid() const override;

    static std::vector<int> controllersInUse;
    static int deviceRemoved;

private:
    SDL_GameController* _controller;

    InputDeviceControlValues _controls;

    RJ::Time _lastDribblerTime;
    RJ::Time _lastKickerTime;

    void openInputDevice();
    void closeInputDevice();
    bool connected;
    int controllerId;
};
