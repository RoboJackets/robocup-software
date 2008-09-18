#ifndef _PACKET__STATUS_HPP_
#define _PACKET__STATUS_HPP_

#include "Ports.hpp"

namespace Packet
{
    class RobotStatus
    {
		public:
			static const int Type = RobotStatusPort;

			RobotStatus() :
				timestamp(0) {}

			class Robot
			{
				public:
					Robot() :
						battery(0), rssi(0), charged(false), ballPresent(false) {}

					/** battery voltage */
					float battery;

					/** received signal strength */
					uint8_t rssi;

					/** true when kicker is charged */
					bool charged;

					/** true if ball is present */
					bool ballPresent;
			} __attribute__((__packed__));

			Robot robots[5];
			uint64_t timestamp;

    } __attribute__((__packed__));
};

#endif // _PACKET__STATUS_HPP_
