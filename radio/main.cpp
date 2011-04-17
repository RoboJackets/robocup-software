// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <assert.h>
#include <poll.h>

#include <string>

#include <Network.hpp>
#include <protobuf/RadioTx.pb.h>
#include <protobuf/RadioRx.pb.h>
#include <Utils.hpp>
#include <QUdpSocket>

#include "Radio.hpp"

using namespace std;
using namespace Packet;

void usage(const char* prog)
{
	fprintf(stderr, "usage: %s [-n i] [-2010] <-y|-b>\n", prog);
	fprintf(stderr, "\t-n: use base station i (0 is the first base station detected by libusb)\n");
	fprintf(stderr, "\t-o: run an opponent team as well (note: radio only handles up to 5 robots)\n");
	fprintf(stderr, "\t--debug_tx: print transmitted packets\n");
	fprintf(stderr, "\t--debug_rx: print received packets\n");
	exit(1);
}

int main(int argc, char* argv[])
{
	bool useOpp;
	bool debug_tx = false;
	bool debug_rx = false;
	
	int n = 0;
	for (int i = 0; i < argc; ++i)
	{
		const char* var = argv[i];

		if (strcmp(var, "--debug_tx") == 0)
		{
			debug_tx = true;
		}
		else if (strcmp(var, "--debug_rx") == 0)
		{
			debug_rx = true;
		}
		else if (strcmp(var, "-n") == 0)
		{
			if (i == (argc - 1))
			{
				fprintf(stderr, "-n must be followed by the base station number\n");
				usage(argv[0]);
			}
		}
		else if (strcmp(var, "-o") == 0)
		{
			useOpp = true;
		}
	}

	if (n)
	{
		fprintf(stderr, "WARNING: Specifying -n will likely result in the wrong base station being used if it is unplugged and reconnected.\n");
	}
	
	// Set up sockets
	QUdpSocket socket[2];
	for (int i = 0; i < 2; ++i)
	{
		if (!socket[i].bind(RadioTxPort + i))
		{
			fprintf(stderr, "Can't bind to port %d\n", RadioTxPort + i);
			return 1;
		}
	}
	
	Radio *radio = 0;
	
	uint8_t reverse_packet[Reverse_Size + 2];
	uint8_t forward_packet[Forward_Size];
	int sequence = 0;
	
	uint64_t lastTime = Utils::timestamp();
	
	bool radioErrorPrinted = false;
	
	RadioTx txPacket[2];
	while (true)
	{
		bool newData[2];
		
		// Make sure we have a radio
		if (!radio)
		{
			try
			{
				radio = new Radio(n);
			} catch (exception &ex)
			{
				if (!radioErrorPrinted)
				{
					fprintf(stderr, "%s\n", ex.what());
					fprintf(stderr, "Waiting for the base station to be connected...\n");
					radioErrorPrinted = true;
				}
				
				usleep(500 * 1000);
				
				// Can't get any farther.  Go back to polling the sockets and radio.
				continue;
			}
			
			fprintf(stderr, "\nRadio connected\n");
			radioErrorPrinted = false;
		}
		
		// Wait for RadioTX packets
		struct pollfd pfd[2];
		pfd[0].fd = socket[0].socketDescriptor();
		pfd[0].events = POLLIN;
		pfd[1].fd = socket[1].socketDescriptor();
		pfd[1].events = POLLIN;
		if (poll(pfd, 2, -1) < 1)
		{
			printf("Poll: %m\n");
			continue;
		}
		
		// Read RadioTx packets
		for (int i = 0; i < 2; ++i)
		{
			newData[i] = false;
			
			while (socket[i].hasPendingDatagrams())
			{
				int n = socket[i].pendingDatagramSize();
				string str(n, 0);
				socket[i].readDatagram(&str[0], n);
				
				if (!txPacket[i].ParseFromString(str))
				{
					fprintf(stderr, "Bad RadioTx packet of %d bytes on channel %d\n", n, i);
				} else {
					newData[i] = true;
				}
			}
		}
		
		//FIXME - Switch between teams for reverse channel
		int reverse_board_id = txPacket[0].reverse_board_id();
		
		// Build a forward packet
		forward_packet[0] = (sequence << 4) | reverse_board_id;
		
		bool printed = false;
		int offset = 1;
		int self_bots = 0;
		int robot_id;
		for (robot_id = 0; robot_id < 5 && robot_id < txPacket[0].robots_size(); ++robot_id)
		{
			//FIXME - Read from both channels and merge
			const RadioTx::Robot &robot = txPacket[0].robots(robot_id);
			int board_id = robot.board_id();
			
			int8_t m0, m1, m2, m3;
			uint8_t kick, roller;
			
			self_bots++;
			
			m1 = -robot.motors(0);
			m2 = -robot.motors(1);
			m3 = -robot.motors(2);
			m0 = -robot.motors(3);
			kick = robot.kick();
			
			if (robot.roller() > 0)
			{
				roller = robot.roller() * 2;
			} else {
				roller = 0;
			}
			
			forward_packet[offset++] = m0;
			forward_packet[offset++] = m1;
			forward_packet[offset++] = m2;
			forward_packet[offset++] = m3;
			forward_packet[offset++] = (roller & 0xf0) | (board_id & 0x0f);
			forward_packet[offset++] = kick;
		}
		
		// Unused slots
		for (; robot_id < 5; ++robot_id)
		{
			forward_packet[offset++] = 0;
			forward_packet[offset++] = 0;
			forward_packet[offset++] = 0;
			forward_packet[offset++] = 0;
			forward_packet[offset++] = 0x0f;
			forward_packet[offset++] = 0;
		}
		
#if 0
		//FIXME - Redesign this in light of the new approach to radio channels
		if (useOpp)
		{	
			offset = 3 + self_bots * 5;
			for (int robot_id = 0; robot_id < 5 - self_bots; ++robot_id)
			{
				const RadioTx::Robot &robot = oppTxPacket.robots(robot_id);
				int board_id = robot.board_id();
				
				int8_t m0, m1, m2, m3;
				uint8_t kick, roller;
				
				m1 = -robot.motors(0);
				m2 = -robot.motors(1);
				m3 = -robot.motors(2);
				m0 = -robot.motors(3);
				kick = robot.kick();
				
				if (robot.roller() > 0)
				{
					roller = robot.roller() * 2;
				} else {
					roller = 0;
				}
				
				forward_packet[offset++] = m0;
				forward_packet[offset++] = m1;
				forward_packet[offset++] = m2;
				forward_packet[offset++] = m3;
				forward_packet[offset++] = (roller & 0xf0) | (board_id & 0x0f);
				forward_packet[offset++] = kick;
			}
		}
#endif

		if (debug_tx)
		{
			uint64_t t1 = Utils::timestamp();
			uint64_t dt = t1 - lastTime;
			lastTime = t1;
			printf("%3ld.%03ldms ", dt / 1000, dt % 1000);
			
			for (unsigned int i = 0; i < Forward_Size; ++i)
			{
				printf("%02x ", forward_packet[i]);
			}
			printed = true;
		}
		
		bool read_ok = false;
		uint64_t rx_time = 0;
		try
		{
			// Send the forward packet
			radio->write_packet(forward_packet, Forward_Size);
			
			// Read a forward packet if one is available
			//FIXME - Why is the timeout 1 instead of 0?
			read_ok = radio->read_packet(reverse_packet, sizeof(reverse_packet), 1);
			rx_time = Utils::timestamp();
		} catch (exception &ex)
		{
			fprintf(stderr, "%s\n", ex.what());
			delete radio;
			radio = 0;
		}
		
		sequence = (sequence + 1) & 15;
		
		if (read_ok)
		{
			// A reverse packet was received
			if (debug_rx)
			{
				if (debug_tx)
				{
					printf("   ");
				}
				printf("rev");
				for (unsigned int i = 0; i < Reverse_Size; ++i)
				{
					printf(" %02x", reverse_packet[i]);
				}
				printed = true;
			}
			
			RadioRx rxPacket;
			int board_id = reverse_packet[0] & 0x0f;
			
			rxPacket.set_timestamp(rx_time);
			rxPacket.set_board_id(board_id);
			rxPacket.set_rssi((int8_t)reverse_packet[1] / 2.0);
			rxPacket.set_battery(reverse_packet[3] * 3.3 / 256.0 * 5.0);
			rxPacket.set_ball(reverse_packet[5] & (1 << 5));
			rxPacket.set_charged(reverse_packet[4] & 1);
			rxPacket.set_motor_fault(reverse_packet[5] & 0x1f);
			
			for (int i = 0; i < 4; ++i)
			{
				int value = reverse_packet[6 + i] | ((reverse_packet[10] >> (i * 2)) & 3) << 8;
				rxPacket.add_encoders(value);
			}
			
			//FIXME - Send on which channel?
			string str;
			rxPacket.SerializeToString(&str);
			socket[0].writeDatagram(&str[0], str.size(), QHostAddress(QHostAddress::LocalHost), RadioRxPort);
		}
		
		if (printed)
		{
			printf("\n");
		}
	}

	return 0;
}
