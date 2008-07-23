#ifndef COMM_DATA_HPP
#define COMM_DATA_HPP

#include <memory>
#include <stdint.h>

#include "Ports.hpp"

namespace Packet
{
	/** Data that is sent from the controller to all robots */
	typedef struct CommData
	{
		static const int Type = CommPort;

        // How the radio process updates status of robots/boards.
        // A robot is a board that is assigned a valid robot ID.
        typedef enum
        {
            Fixed,          // Updates status on a single robot
            ScanRobots,     // Cycles through all robots
        } StatusMode;

		typedef struct Robot
		{
			Robot()
			{
                valid = false;
				roller = 0;
				memset(motor, 0, sizeof(motor));
				kick = 0;
                oneTouch = false;
			}

			/** speed of the roller
             * Positive is dribbling.
             * Negative is anti-dribbling.
             */
			int8_t roller;

			/** true means kick */
			uint8_t kick;

			//motor speeds
			int8_t motor[4];
            
            bool oneTouch;

			//whether the robot is valid or not
			bool valid;
		} __attribute__((__packed__)) Robot;

		CommData()
        {
            timestamp = 0;
            mode = ScanRobots;
            robot = 0;
        }

		void invalidate()
		{
			for (unsigned int i=0 ; i<5 ; ++i)
			{
				robots[i].valid = false;
			}
		}

        uint64_t timestamp;

		Robot robots[5];

        // Status reporting mode
        StatusMode mode;

        // Robot number if mode==Fixed.
        int robot;
	} __attribute__((__packed__)) CommData;
}

#endif
