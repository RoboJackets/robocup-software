#include <stdio.h>
#include <string.h>

#include <Network/Network.hpp>
#include <Network/Sender.hpp>
#include <RadioTx.hpp>

void usage(const char* prog)
{
	printf("usage: %s <-y|-b>\n", prog);
	printf("\t-y: run as the yellow team\n");
	printf("\t-b: run as the blue team\n");
}

int main(int argc, char* argv[])
{
	Team team = UnknownTeam;

	for (int i=0 ; i<argc; ++i)
	{
		const char* var = argv[i];

		if (strcmp(var, "-y") == 0)
		{
			team = Yellow;
		}
		else if (strcmp(var, "-b") == 0)
		{
			team = Blue;
		}
	}

	if (team == UnknownTeam)
	{
		printf("Error: No team specified\n");
		usage(argv[0]);
		return 0;
	}
	
	Network::Sender sender(Network::Address, Network::addTeamOffset(team, Network::RadioTx));
	
	while (true)
	{
		//from control program -> radio program
		Packet::RadioTx txPacket;
		
		//populate packet
		
		sender.send(txPacket);
		
		//TODO wait...
	}
	
	return 0;
}
