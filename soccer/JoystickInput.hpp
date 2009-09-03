#pragma once

#include <linux/joystick.h>

#include <stdexcept>
#include <cerrno>

#include <stdint.h>

class JoystickInput
{
	public:
	    JoystickInput(const char* filename) throw (std::runtime_error);
	    ~JoystickInput();

	    bool poll(int timeout = -1);

	    int8_t* axis;
	    int8_t* button;

	private:
	    // File handle of joystick device
	    int _fd;

	    JoystickInput(JoystickInput&);
	    JoystickInput& operator=(JoystickInput&);
};
