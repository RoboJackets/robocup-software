#pragma once

#include <protobuf/RadioTx.pb.h>

#include <QMutex>
#include <boost/utility.hpp>
#include <stdexcept>
#include <stdint.h>
#include <vector>

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
		
		void update();
		void drive(Packet::RadioTx::Robot *tx);

	private:
		bool open();
		void close();
		
		QMutex _mutex;
		
		// File handle of joystick device
		int _fd;
		
		std::vector<int> _axis;
		std::vector<int> _button;

		bool _autonomous;
		int _roller;
		int _stored_roller;
		
		// D-pad
		bool dUp()    const { return (_axis[5] < 0) ? true : false; }
		bool dDown()  const { return (_axis[5] > 0) ? true : false; }
		bool dRight() const { return (_axis[4] > 0) ? true : false; }
		bool dLeft()  const { return (_axis[4] < 0) ? true : false; }
};
