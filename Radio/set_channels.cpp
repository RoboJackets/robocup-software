#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Radio.hpp"

void usage()
{
	printf("Usage: set_channels <device> <rx> <tx>\n");
	printf("Channels are 0-100 or \"off\"\n");
	exit(1);
}

int channel(const char *str)
{
	if (!strcmp(str, "off"))
	{
		return Radio::Off;
	} else {
		int ch = atoi(str);
		if (ch < 0 || ch > 100)
		{
			usage();
		}
		
		return ch;
	}
}

int main(int argc, char *argv[])
{
	if (argc != 4)
	{
		usage();
	}
	
	int rx = channel(argv[2]);
	int tx = channel(argv[3]);
	
	Radio radio(argv[1]);
	radio.command_mode(true);
	radio.set_channels(rx, tx);
	radio.command_mode(false);
	
	return 0;
}
