#include "Joystick.hpp"

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

static const uint64_t Dribble_Step_Time = 125 * 1000;
static const uint64_t Kicker_Step_Time = 125 * 1000;

// rates used by damping settings
// Rotation in rad/s
static const float Rotation_Max_Speed = 4 * M_PI;
static const float Rotation_Max_Damped_Speed = 1 * M_PI;

// Translation in m/s
static const float Translation_Max_Speed = 3.0;
static const float Translation_Max_Damped_Speed = 1.0;

static const char *devices[] =
{
	"/dev/input/by-id/usb-Logitech_Logitech_Cordless_RumblePad_2-joystick",
	"/dev/input/by-id/usb-Logitech_Logitech_Dual_Action-joystick",
	
	// End of list
	0
};

Joystick::Joystick():
	_mutex(QMutex::Recursive)
{
	_dampedRotation = true;
	_dampedTranslation = true;
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

Joystick::~Joystick()
{
	close();
}

bool Joystick::open()
{
	QMutexLocker locker(&_mutex);
	
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
			printf("Joystick: %s\n", devices[i]);
			
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

void Joystick::close()
{
	QMutexLocker locker(&_mutex);
	if (_fd >= 0)
	{
		::close(_fd);
		_fd = -1;
		
		_axis.clear();
		_button.clear();
	}
}

void Joystick::update()
{
	QMutexLocker locker(&_mutex);
	
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
			return;
		} else if (ret < 0)
		{
			// Poll error (not fd error)
			//
			// EINTR can happen while profiling because the poll() is interrupted
			// by the timer signal used for statistical sampling.
			// In this case, ignore the error and try again next time.
			if (errno != EINTR)
			{
				printf("Joystick poll error: %m\n");
				close();
			}
			return;
		}
		
		if (pfd.revents & POLLHUP)
		{
			// Joystick disconnected
			printf("Lost joystick\n");
			close();
			return;
		}
		
		struct js_event event;
		if (read(_fd, &event, sizeof(event)) != sizeof(event))
		{
			return;
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
}

void Joystick::drive(RadioTx::Robot *tx)
{
	QMutexLocker locker(&_mutex);
	
	if (_axis.size() < 4 || _button.size() < 8)
	{
		// No joystick or wrong joystick
		return;
	}
	
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
	
	if (_dampedTranslation)
	{
		tx->set_body_x(input.y  * Translation_Max_Damped_Speed);
		tx->set_body_y(-input.x * Translation_Max_Damped_Speed);
	} else
	{
		tx->set_body_x(input.y  * Translation_Max_Speed);
		tx->set_body_y(-input.x * Translation_Max_Speed);
	}

	if (_dampedRotation)
		tx->set_body_w(-leftX * Rotation_Max_Damped_Speed);
	else
		tx->set_body_w(-leftX * Rotation_Max_Speed);
	
	if (_button[6])
	{
		_dribblerOn = false;
	} else if (_button[4])
	{
		_dribblerOn = true;
	}
	
	uint64_t now = timestamp();
	if (_button[1])
	{
		if (_dribbler > 0 && (now - _lastDribblerTime) >= Dribble_Step_Time)
		{
			_dribbler -= 8;
			_lastDribblerTime = now;
		}
	} else if (_button[3])
	{
		if (_dribbler < 120 && (now - _lastDribblerTime) >= Dribble_Step_Time)
		{
			_dribbler += 8;
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
			_kicker -= 8;
			_lastKickerTime = now;
		}
	} else if (_button[2])
	{
		if (_kicker < 247 && (now - _lastKickerTime) >= Kicker_Step_Time)
		{
			_kicker += 8;
			_lastKickerTime = now;
		}
	} else {
		_lastKickerTime = now - Kicker_Step_Time;
	}

	tx->set_dribbler(_dribblerOn ? _dribbler : 0);
	tx->set_kick((_button[7] | _button[5]) ? _kicker : 0);
	tx->set_use_chipper(_button[5]);
	tx->set_kick_immediate((_button[7] | _button[5]) ? 1 : 0);

	if(_button[9])
		tx->set_sing(true);
	if(_button[8])
		tx->set_anthem(true);

#define print_cmd 0
#if print_cmd
	printf("Dribbler %d \n",_dribbler);
	printf("Kick %d \n",_kicker);
#endif
}

JoystickControlValues Joystick::getJoystickControlValues()
{
	QMutexLocker locker(&_mutex);
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
	
	if (_dampedTranslation)
	{
		vals.bodyX = input.y  * Translation_Max_Damped_Speed;
		vals.bodyY = -input.x * Translation_Max_Damped_Speed;
	} else
	{
		vals.bodyX = input.y  * Translation_Max_Speed;
		vals.bodyY = -input.x * Translation_Max_Speed;
	}

	if (_dampedRotation)
		vals.bodyW = -leftX * Rotation_Max_Damped_Speed;
	else
		vals.bodyW = -leftX * Rotation_Max_Speed;

	vals.kickPower = _kicker;
	vals.dribblerPower = _dribbler;
	vals.kick = _button[7] | _button[5];
	vals.dribble = _dribblerOn;
	vals.chip = _button[5];

	return vals;
}

void Joystick::reset()
{
	QMutexLocker locker(&_mutex);
	_dribblerOn = false;
}

void Joystick::dampedRotation(bool value)
{
	QMutexLocker locker(&_mutex);
	_dampedRotation = value;
}

void Joystick::dampedTranslation(bool value)
{
	QMutexLocker locker(&_mutex);
	_dampedTranslation = value;
}

