#include "JoystickInput.hpp"

#include <unistd.h>
#include <fcntl.h>

JoystickInput::JoystickInput(const char* filename)
{
	_fd = open(filename, O_RDONLY);
    if (_fd < 0)
    {
    	throw std::runtime_error(strerror(errno));
    }

    //fcntl(_fd, F_SETFL, O_NONBLOCK );

    unsigned int numAxes = 0, numButtons = 0;
    ioctl(_fd, JSIOCGAXES, &numAxes );
    ioctl(_fd, JSIOCGBUTTONS, &numButtons );

    axis = new int8_t[numAxes];
    button = new int8_t[numButtons];

    //printf("Number of axis: %u ", (unsigned int)sizeof(axis));
    memset(axis, 0, numAxes);
    memset(button, 0, numButtons);
}

JoystickInput::~JoystickInput()
{
	if (_fd >= 0)
	{
		close(_fd);
		_fd = -1;
	}

	delete[] axis;
	delete[] button;
}

bool JoystickInput::poll(int timeout)
{
	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(_fd, &fds);

	struct timeval tv, *ptv;

	if (timeout >= 0)
	{
		tv.tv_usec = timeout * 1000;
		tv.tv_sec = timeout / 1000;

		ptv = &tv;
	}
	else
	{
		ptv = 0;
	}

	if (select(_fd + 1, &fds, NULL, NULL, ptv) != 1)
	{
		return false;
	}

	struct js_event event;
	if (read(_fd, &event, sizeof(event)) != sizeof(event))
	{
		return false;
	}

	switch (event.type)
	{
		case JS_EVENT_BUTTON:
			button[event.number] = event.value;
			return true;

		case JS_EVENT_AXIS:
			axis[event.number] = event.value >> 8;
			return true;
		//case JS_EVENT_INIT:
		default:
			return false;
	}
}
