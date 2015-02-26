#pragma once

#include "Joystick.hpp"

#include <SDL/SDL.h>

/**
 * @brief Logitecch Gamepad/Joystick used to control robots
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
        SDL_Joystick *_joystick;

        JoystickControlValues _controls;

        Time _lastDribblerTime;
        Time _lastKickerTime;
};
