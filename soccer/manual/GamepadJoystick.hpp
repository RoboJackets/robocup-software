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
    GamepadJoystick(SDL_Event& event);
    ~GamepadJoystick();

    static void initDeviceType();

    void reset() override;
    void update(SDL_Event& event) override;
    InputDeviceControlValues getInputDeviceControlValues() override;

    void openInputDevice(SDL_Event& event);
    void closeInputDevice();

    bool valid() const override;

    static std::vector<int> joysticksInUse;
    static int deviceRemoved;


private:
    SDL_Joystick* _joystick;

    InputDeviceControlValues _controls;

    RJ::Time _lastDribblerTime;
    RJ::Time _lastKickerTime;

    bool connected;
    int joystickId;

};
