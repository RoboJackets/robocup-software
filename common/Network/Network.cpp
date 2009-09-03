#include "Network.hpp"

const char* Network::Address = "226.0.0.1"; 
const char* Network::RefboxAddress = "224.5.23.1";

uint16_t Network::addTeamOffset(Team t, uint16_t portOffset)
{
	if (t == Blue)
	{
		return Network::BluePortBase + portOffset;
	}
	
	return Network::YellowPortBase + portOffset;
}
