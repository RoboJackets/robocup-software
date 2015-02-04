#include "GamepadJoystick.hpp"

#include <iostream>
#include <QMutexLocker>
#include <linux/joystick.h>
#include <Geometry2d/Point.hpp>
#include <Utils.hpp>

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>

using namespace Packet;
using namespace std;

static const Time Dribble_Step_Time = 125 * 1000;
static const Time Kicker_Step_Time = 125 * 1000;


static const char *devices[] =
{
    "/dev/input/by-id/usb-Logitech_Logitech_Cordless_RumblePad_2-joystick",
    "/dev/input/by-id/usb-Logitech_Logitech_Dual_Action-joystick",
    
    // End of list
    0
};

GamepadJoystick::GamepadJoystick() {
    _dribbler = 0;
    _dribblerOn = false;
    _kicker = 0;
    _lastDribblerTime = 0;
    _lastKickerTime = 0;
    _fd = -1;
    
    if (!open())
    {
        printf("No joystick\n");
    }
}

GamepadJoystick::~GamepadJoystick()
{
    close();
}

bool GamepadJoystick::valid() const
{
    return _fd >= 0;
}

bool GamepadJoystick::open()
{
    QMutexLocker locker(&mutex());
    
    if (_fd >= 0)
    {
        // Already open
        return true;
    }
    
    reset();
    
    for (int i = 0; devices[i]; ++i)
    {
        _fd = ::open(devices[i], O_RDONLY);
        if (_fd >= 0)
        {
            printf("GamepadJoystick: %s\n", devices[i]);
            
            unsigned int numAxes = 0, numButtons = 0;
            ioctl(_fd, JSIOCGAXES, &numAxes);
            ioctl(_fd, JSIOCGBUTTONS, &numButtons);

            _axis.resize(numAxes);
            _button.resize(numButtons);
            
            return true;
        }
    }
    
    return false;
}

void GamepadJoystick::close()
{
    QMutexLocker locker(&mutex());
    if (_fd >= 0)
    {
        ::close(_fd);
        _fd = -1;
        
        _axis.clear();
        _button.clear();
    }
}

void GamepadJoystick::update()
{
    QMutexLocker locker(&mutex());
    
    if (_fd < 0 && !open())
    {
        // No joystick available
        return;
    }
    
    while (true)
    {
        struct pollfd pfd;
        
        pfd.fd = _fd;
        pfd.events = POLLIN | POLLHUP;
        int ret = poll(&pfd, 1, 0);
        if (ret == 0)
        {
            // No data
            break;
        } else if (ret < 0)
        {
            // Poll error (not fd error)
            //
            // EINTR can happen while profiling because the poll() is interrupted
            // by the timer signal used for statistical sampling.
            // In this case, ignore the error and try again next time.
            if (errno != EINTR)
            {
                printf("GamepadJoystick poll error: %m\n");
                close();
            }
            return;
        }
        
        if (pfd.revents & POLLHUP)
        {
            // GamepadJoystick disconnected
            printf("Lost joystick\n");
            close();
            return;
        }
        
        struct js_event event;
        if (read(_fd, &event, sizeof(event)) != sizeof(event))
        {
            break;
        }

        if (event.type == JS_EVENT_BUTTON)
        {
            _button[event.number] = event.value;
        }
        else if (event.type == JS_EVENT_AXIS)
        {
            // Store analog data
            _axis[event.number] = event.value;
        }
    }



    if (_button[6])
    {
        _dribblerOn = false;
    } else if (_button[4])
    {
        _dribblerOn = true;
    }


    Time now = timestamp();
    if (_button[1])
    {
        if (_dribbler > 0 && (now - _lastDribblerTime) >= Dribble_Step_Time)
        {
            _dribbler -= 0.1;
            _lastDribblerTime = now;
        }
    } else if (_button[3])
    {
        if (_dribbler < 1 && (now - _lastDribblerTime) >= Dribble_Step_Time)
        {
            _dribbler += 0.1;
            _lastDribblerTime = now;
        }
    } else {
        // Let dribbler speed change immediately
        _lastDribblerTime = now - Dribble_Step_Time;
    }

    if (_button[0])
    {
        if (_kicker > 0 && (now - _lastKickerTime) >= Kicker_Step_Time)
        {
            _kicker -= 0.1;
            _lastKickerTime = now;
        }
    } else if (_button[2])
    {
        if (_kicker < 1 && (now - _lastKickerTime) >= Kicker_Step_Time)
        {
            _kicker += 0.1;
            _lastKickerTime = now;
        }
    } else {
        _lastKickerTime = now - Kicker_Step_Time;
    }
}

//  note: we used to have buttons for singing/anthem
//     if(_button[9])
//         tx->set_sing(true);
//     if(_button[8])
//         tx->set_anthem(true);

JoystickControlValues GamepadJoystick::getJoystickControlValues()
{
    QMutexLocker locker(&mutex());
    JoystickControlValues vals;

    // applying dampening - scales each to [-1,1] range
    float leftX =   _axis[Axis_Left_X]  / 32768.0f;
    float rightX =  _axis[Axis_Right_X] / 32768.0f;
    float rightY = -_axis[Axis_Right_Y] / 32768.0f;
    
    //input is vx, vy in robot space
    Geometry2d::Point input(rightX, rightY);

    //if using DPad, this is the input value
    float mVal = fabs(rightY);

    if (dUp())
    {
        input.y = mVal;
        input.x = 0;
    }
    else if (dDown())
    {
        input.y = -mVal;
        input.x = 0;
    }
    else if (dRight())
    {
        input.y = 0;
        input.x = mVal;
    }
    else if (dLeft())
    {
        input.y = 0;
        input.x = -mVal;
    }

    vals.translation = Geometry2d::Point(input.x, input.y);

    vals.rotation = -leftX;

    vals.kickPower = _kicker;
    vals.dribblerPower = _dribbler;
    vals.kick = _button[7] | _button[5];
    vals.dribble = _dribblerOn;
    vals.chip = _button[5];

    return vals;
}

void GamepadJoystick::reset()
{
    QMutexLocker locker(&mutex());
    _dribblerOn = false;
}
