#pragma once

#include <Team.h>
#include <stdint.h>

/** network information for inter process communication. 
 *  See the networking information on the wiki about how processes communicate */
namespace Network
{
	/** address that all packets are sent to */
	extern const char* Address;
	
	// base port numbers for each team
	const uint16_t YellowPortBase = 20000;
	const uint16_t BluePortBase = 30000;
	
	/** global vision data port */
	const uint16_t Vision = 1337;
    
    /** Port for commands to the simulator */
    const uint16_t SimCommandPort = 1339;
	
	/** Data going to the radio application from the control apps */
	const uint16_t RadioTx = 1;
	
	/** Data comming from the radio application to the control apps */
	const uint16_t RadioRx = 2;
	
	extern const char* RefboxAddress;
	const uint16_t RefboxPort = 10001;
	
	uint16_t addTeamOffset(Team t, uint16_t portOffset);
}
