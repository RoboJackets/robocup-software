#pragma once

#include <linux/joystick.h>
#include <boost/utility.hpp>

#include <stdexcept>

#include <stdint.h>
#include <vector>

class SystemState;

class JoystickInput: boost::noncopyable
{
	public:
	    JoystickInput(const char* filename, SystemState *state) throw (std::runtime_error);
	    ~JoystickInput();

		bool valid() const
		{
			return _fd >= 0;
		}
		
		bool poll(int timeout = 0);
		void drive();

		std::vector<int> axis;
		std::vector<int> button;

	private:
	    // File handle of joystick device
		int _fd;
		
		bool _kick;
		int _roller;
		int _stored_roller;
		
		// D-pad
		bool dUp() const { return (axis[5] < 0) ? true : false; }
		bool dDown() const { return (axis[5] > 0) ? true : false; }
		bool dRight() const { return (axis[4] > 0) ? true : false; }
		bool dLeft() const { return (axis[4] < 0) ? true : false; }
		
		SystemState *_state;
};
