#ifndef NETWORK_HPP_
#define NETWORK_HPP_

#include <Team.h>

/** network information for inter process communication. 
 *  See the networking information on the wiki about how processes communicate */
namespace Network
{
	/** address that all packets are sent to */
	const char* Address = "226.0.0.1"; 
	
	// base port numbers for each team
	uint16_t YellowPortBase = 20000;
	uint16_t BluePortBase = 30000;
	
	/** global vision data port */
	uint16_t Vision = 1337;
	
	/** Data going to the radio application from the control apps */
	uint16_t RadioTx = 1;
	
	/** Data comming from the radio application to the control apps */
	uint16_t RadioRx = 2;
	
	const char* RefboxAddress = "224.5.23.1";
	uint16_t RefboxPort = 10001;
	
	uint16_t addTeamOffset(Team t, uint16_t portOffset)
	{
		if (t == Blue)
		{
			return Network::BluePortBase + portOffset;
		}
		
		return Network::YellowPortBase + portOffset;
	}
}

#endif /* NETWORK_HPP_ */
