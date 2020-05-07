#pragma once

#include "InputDevice.hpp"

#include <SDL.h>

/**
 * @brief Logitecch Gamepad/Joystick used to control robots
 */
class GamepadController : public InputDevice {
public:
    GamepadController();
    GamepadController(SDL_Event& event);
    ~GamepadController();

    // static void initDeviceType();

    void reset() override;
    void update(SDL_Event& event) override;
    InputDeviceControlValues getInputDeviceControlValues() override;

    void openInputDevice(SDL_Event& event);
    void closeInputDevice();

    bool valid() const override;

    static std::vector<int> controllersInUse;
    static int deviceRemoved;

private:
    SDL_GameController* _controller;

    InputDeviceControlValues _controls;

    RJ::Time _lastDribblerTime;
    RJ::Time _lastKickerTime;

    bool connected;
    int controllerId;
};
