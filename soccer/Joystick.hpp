#pragma once

#include <protobuf/RadioTx.pb.h>

#include <QMutex>
#include <boost/utility.hpp>
#include <stdexcept>
#include <stdint.h>
#include <vector>
/**
 * this is the class for the joystick control of the robots
 */
class Joystick: boost::noncopyable
{
	public:
		Joystick();
		~Joystick();

		bool valid() const
		{
			return _fd >= 0;
		}
		
		bool autonomous() const
		{
			return _autonomous;
		}
		
		void reset();
		void update();
		void drive(Packet::RadioTx::Robot *tx);

		void dampedRotation(bool value);
		void dampedTranslation(bool value);

	private:
		bool open();
		void close();
		
		QMutex _mutex;
		
		// File handle of joystick device
		int _fd;
		
		std::vector<int> _axis;
		std::vector<int> _button;

		bool _autonomous;
		int _dribbler;
		bool _dribblerOn;
		
		int _kicker;

		bool _dampedRotation, _dampedTranslation;

		// Last time the dribbler speed changed
		uint64_t _lastDribblerTime;
		uint64_t _lastKickerTime;
		
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
