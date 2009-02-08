#ifndef GAMEPAD_HPP
#define GAMEPAD_HPP

#include "JoystickInput.hpp"

/** Logitec Gamepad class for easy access to buttons and axes 
 * from a HidInput device */
class Gamepad : private JoystickInput
{
	typedef int8_t axis_t;
	typedef bool key_t;
	
	public:
		Gamepad(const char* file) : JoystickInput(file) {}
		~Gamepad() {}
		
		bool waitForInput(int timeout = -1) { return this->poll(timeout); }
		
		//left side
		inline axis_t lX() const { return clip_positive(axis[0]); }
		inline axis_t lY() const { return clip_negative(axis[1]); }
		inline key_t lTab() const { return b5(); }
		inline key_t lTrigger() const { return b7(); }
		
		//rigth side
		inline axis_t rX() const { return clip_positive(axis[2]); }
		inline axis_t rY() const { return clip_negative(axis[3]); }
		inline key_t rTab() const { return b6(); }
		inline key_t rTrigger() const { return b8(); }
		
		//dpad
		inline key_t dUp() const { return (axis[5] < 0) ? true : false; }
		inline key_t dDown() const { return (axis[5] > 0) ? true : false; }
		inline key_t dRight() const { return (axis[4] > 0) ? true : false; }
		inline key_t dLeft() const { return (axis[4] < 0) ? true : false; }
		
		//buttoms
		inline key_t b1() const { return button[0]; }
		inline key_t b2() const { return button[1]; }
		inline key_t b3() const { return button[2]; }
		inline key_t b4() const { return button[3]; }
		inline key_t b5() const { return button[4]; }
		inline key_t b6() const { return button[5]; }
		inline key_t b7() const { return button[6]; }
		inline key_t b8() const { return button[7]; }
		inline key_t b9() const { return button[8]; }
		inline key_t b10() const { return button[9]; }
		
	private:
        static axis_t clip_positive(axis_t x)
        {
            if (x == -128)
            {
                return -127;
            } else {
                return x;
            }
        }
        
        static axis_t clip_negative(axis_t x)
        {
            if (x == -128)
            {
                return 127;
            } else {
                return -x;
            }
        }
        
        Gamepad(Gamepad&);
		Gamepad& operator=(Gamepad&);
};

#endif //GAMEPAD_HPP
