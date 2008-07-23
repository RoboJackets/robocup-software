#include <QCoreApplication>

#include <Packet/PacketReceiver.hpp>

#include "Ref.hpp"

Ref* yellow = 0;
Ref* blue = 0;

/** if true, print debug info */
bool debug = false;

void usage(const char* prog)
{
	printf("Usage: %s [-b] [-y] [-debug]\n", prog);
	printf("\t-b: enable ref for blue team\n");
	printf("\t-y: enable ref for yellow team\n");
	printf("\t-y: enable ref for yellow team\n");
	printf("\t-debug: enable debug output\n");
}

void refPacketHandler(const RefPacket* data)
{
    if (!data)
    {
        return;
    }
    
	if (debug && data->cmd >= ' ')
	{
		printf("Counter: %2d\tBlue: %2d\tYellow: %2d\tCmd: %c\n",
				data->counter, data->goalsYellow, data->goalsBlue,
				data->cmd);
	}
	
	if (yellow)
	{
		yellow->refPacketHandler(data);
	}
	
	if (blue)
	{
		blue->refPacketHandler(data);
	}
}

int main(int argc, char *argv[])
{
	QCoreApplication a(argc, argv);
	
	bool startYellow = false, startBlue = false;
	
	int c = 0;
	while (++c < argc)
	{
		if (strncmp(argv[c], "-b", 2) == 0)
		{
			startBlue = true;
		}
		else if (strncmp(argv[c], "-y", 2) == 0)
		{
			startYellow = true;
		}
		else if (strncmp(argv[c], "-debug", 6) == 0)
		{
			debug = true;
		}
	}
	
	if (!startBlue && !startYellow)
	{
		usage(argv[0]);
		return 1;
	}
	
	Packet::PacketReceiver packetRecv;
	
	//central ref cmd receiver
	packetRecv.addType(QHostAddress("224.5.23.1"), 10001, refPacketHandler);
	
	printf("Ref Started...\n");
	
	if (startYellow)
	{
		printf("Proc Yellow\n");
		yellow = new Ref(Yellow);
		packetRecv.addType(Yellow, yellow, &Ref::visionHandler);
	}
	
	if (startBlue)
	{
		printf("Proc Blue\n");
		blue = new Ref(Blue);
		packetRecv.addType(Blue, blue, &Ref::visionHandler);
	}
	
	while (true)
	{
		packetRecv.receive();
		
		if (yellow)
		{
			yellow->proc();
		}
		
		if (blue)
		{
			blue->proc();
		}
	}
	
	return a.exec();
}
