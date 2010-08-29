#include "Joystick.hpp"

#include <QMutexLocker>
#include <linux/joystick.h>
#include <Geometry2d/Point.hpp>

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
// #include <string.h>
#include <poll.h>
#include <errno.h>

using namespace Packet;

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
	_autonomous = true;
	_roller = 0;
	_stored_roller = 0;
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
			
			if (event.value)
			{
				// Button press
				int n = event.number + 1;
				if (n == 1)
				{
					_autonomous = true;
				} else if (n == 3)
				{
					_autonomous = false;
				}
			}
		} else if (event.type == JS_EVENT_AXIS)
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
	
	int leftX = _axis[0] / 256;
	int leftY = -_axis[1] / 256;
	int rightX = _axis[2] / 256;
	int rightY = -_axis[3] / 256;
	
	//input is vx, vy in robot space
	Geometry2d::Point input(rightX, rightY);

	//if using DPad, this is the input value
	uint8_t mVal = 10 + (int8_t) (abs(rightY));

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

	int max = 0;

	static const Geometry2d::Point axles[4] =
	{
		Geometry2d::Point(1, -1),
		Geometry2d::Point(-1, -1),
		Geometry2d::Point(-1, 1),
		Geometry2d::Point(1, 1)
	};
	
	int motors[4] =	{ 0, 0, 0, 0 };
	for (unsigned int i = 0; i < 4; ++i)
	{
		motors[i] = axles[i].perpCW().dot(input);
		motors[i] -= leftX;

		if (abs(motors[i]) > max)
		{
			max = abs(motors[i]);
		}
	}

	float scale = 1.0f;
	if (max > 127)
	{
		scale = 127.0f / max;
	}

	for (unsigned int i = 0; i < 4; ++i)
	{
		tx->set_motors(i, int8_t(scale * motors[i]));
	}

	if (_button[6])
	{
		_roller = leftY;

		if (_button[4])
		{
			_stored_roller = leftY;
		}
	}

	if (_button[4])
	{
		_roller = _stored_roller;
	}

	tx->set_roller(_roller);
	tx->set_kick(_button[7] ? 255 : 0);
}
