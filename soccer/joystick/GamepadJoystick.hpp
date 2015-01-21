#pragma once

#include "Joystick.hpp"


/**
 * @brief Logitecch Gamepad/Joystick used to control robots
 */
class GamepadJoystick : public Joystick {
    public:
        GamepadJoystick();
        ~GamepadJoystick();

        void reset();
        void update();
        JoystickControlValues getJoystickControlValues();

        bool valid() const;

    private:
        bool open();
        void close();
        
        // File handle of joystick device
        int _fd;
        
        std::vector<int> _axis;
        std::vector<int> _button;

        float _dribbler;
        bool _dribblerOn;
        
        float _kicker;

        // Last time the dribbler speed changed
        Time _lastDribblerTime;
        Time _lastKickerTime;
        
        static const int Axis_Left_X = 0;
        static const int Axis_Left_Y = 1;
        static const int Axis_Right_X = 2;
        static const int Axis_Right_Y = 3;
        static const int Axis_DPad_X = 4;
        static const int Axis_DPad_Y = 5;
        
        // D-pad
        bool dUp()    const { return (_axis[5] < 0) ? true : false; }
        bool dDown()  const { return (_axis[5] > 0) ? true : false; }
        bool dRight() const { return (_axis[4] > 0) ? true : false; }
        bool dLeft()  const { return (_axis[4] < 0) ? true : false; }
};
