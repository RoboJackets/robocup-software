#include "JoystickInput.hpp"

#include <framework/SystemState.hpp>
#include <Constants.hpp>

#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <poll.h>
#include <errno.h>

using namespace Geometry2d;

JoystickInput::JoystickInput(const char* filename, SystemState *state) throw (std::runtime_error)
{
	_state = state;
	_kick = false;
	
	_fd = open(filename, O_RDONLY);
	
	if (_fd >= 0)
	{
		unsigned int numAxes = 0, numButtons = 0;
		ioctl(_fd, JSIOCGAXES, &numAxes);
		ioctl(_fd, JSIOCGBUTTONS, &numButtons);

		axis.resize(numAxes);
		button.resize(numButtons);
	}
}

JoystickInput::~JoystickInput()
{
	if (_fd >= 0)
	{
		close(_fd);
		_fd = -1;
	}
}

bool JoystickInput::poll(int timeout)
{
	if (_fd < 0)
	{
		return false;
	}
	
	struct pollfd pfd;
	
	pfd.fd = _fd;
	pfd.events = POLLIN;
	
	if (::poll(&pfd, 1, timeout) == 0)
	{
		// Timed out
		return false;
	}
	
	struct js_event event;
	if (read(_fd, &event, sizeof(event)) != sizeof(event))
	{
		return false;
	}

	if (event.type == JS_EVENT_BUTTON)
	{
		button[event.number] = event.value;
		
		if (event.value)
		{
			// Button press
			int n = event.number + 1;
			if (n == 1)
			{
				// Select robot
				_state->autonomous = false;
				int oldID = _state->manualID;
				do
				{
				    _state->manualID = (_state->manualID + 1) % Constants::Robots_Per_Team;
				} while (_state->manualID != oldID && !_state->self[_state->manualID].valid);
				
				if (_state->manualID == Constants::Robots_Per_Team)
				{
					_state->manualID = 0;
				}
				_kick = false;
			} else if (n == 2)
			{
				// Deselect
				_state->autonomous = false;
				_state->manualID = -1;
			} else if (n == 3)
			{
				// Auto/manual
				_state->autonomous = !_state->autonomous;
				
				// Clear the manual robot selection so a robot doesn't start moving
				_state->manualID = -1;
			}
		}
	} else if (event.type == JS_EVENT_AXIS)
	{
		axis[event.number] = event.value;
	}
	
	return true;
}

void JoystickInput::drive()
{
	if (_state->manualID < 0 || _state->manualID >= Constants::Robots_Per_Team)
	{
		return;
	}
	
	int leftX = axis[0] / 256;
	int leftY = -axis[1] / 256;
	int rightX = axis[2] / 256;
	int rightY = -axis[3] / 256;
	
	Packet::RadioTx::Robot &tx = _state->self[_state->manualID].radioTx;
	
	tx.set_valid(true);

	//input is vx, vy in robot space
	Point input(rightX, rightY);

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

	static const Point axles[4] =
	{
		Point(1, -1),
		Point(-1, -1),
		Point(-1, 1),
		Point(1, 1)
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
		tx.set_motors(i, int8_t(scale * motors[i]));
	}

	if (button[6])
	{
		_roller = leftY;

		if (button[4])
		{
			_stored_roller = leftY;
		}
	}

	if (button[4])
	{
		_roller = _stored_roller;
	}

	tx.set_roller(_roller);

	tx.set_kick(button[7] ? 255 : 0);
}
