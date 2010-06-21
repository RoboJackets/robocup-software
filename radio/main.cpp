// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <assert.h>

#include <map>

#include <Network/Network.hpp>
#include <Network/PacketReceiver.hpp>
#include <Network/Sender.hpp>
#include <RadioTx.hpp>
#include <RadioRx.hpp>
#include <Utils.hpp>

#include "Radio.hpp"

using namespace std;

Team team = UnknownTeam;

bool useOpp;

Packet::RadioTx txPacket;
Packet::RadioTx oppTxPacket;

void usage(const char* prog)
{
	fprintf(stderr, "usage: %s [-n i] [-2010] <-y|-b>\n", prog);
	fprintf(stderr, "\t-y: run as the yellow team\n");
	fprintf(stderr, "\t-b: run as the blue team\n");
	fprintf(stderr, "\t-n: use base station i (0 is the first base station detected by libusb)\n");
	fprintf(stderr, "\t-o: run an opponent team as well (note: radio only handles up to 5 robots)\n");
	fprintf(stderr, "\t--debug_tx: print transmitted packets\n");
	fprintf(stderr, "\t--debug_rx: print received packets\n");
	exit(1);
}

void packetHandler(const Packet::RadioTx* packet)
{
	txPacket = *packet;
}

void oppPacketHandler(const Packet::RadioTx* packet)
{
	oppTxPacket = *packet;
}

int main(int argc, char* argv[])
{
	bool debug_tx = false;
	bool debug_rx = false;
	
	int n = 0;
	for (int i = 0; i < argc; ++i)
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
		else if (strcmp(var, "--debug_tx") == 0)
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
	
	Radio *radio = 0;
	
	if (team == UnknownTeam)
	{
		fprintf(stderr, "Error: No team specified\n");
		usage(argv[0]);
	}
	
	Network::Sender sender(Network::Address, Network::addTeamOffset(team, Network::RadioRx));
	
	//sender for opponent robots
	Network::Sender oppSender(Network::Address, Network::addTeamOffset(team, Network::RadioRx));

	uint8_t reverse_packet[Reverse_Size + 2];
	
	Network::PacketReceiver receiverSelf;
	Network::PacketReceiver receiverOpp;
	
	receiverSelf.addType(Network::Address, Network::addTeamOffset(team, Network::RadioTx), packetHandler);
	
	if (useOpp)
	{
		receiverOpp.addType(Network::Address, Network::addTeamOffset(opponentTeam(team), Network::RadioTx), oppPacketHandler);
	}

	uint8_t forward_packet[Forward_Size];
	int sequence = 0;
	
	uint64_t lastTime = Utils::timestamp();
	
	while (true)
	{
		bool printed = false;
		
		//clear the incoming packet for the first team only
		txPacket = Packet::RadioTx();
		
		//block for self
		receiverSelf.receive(true);
		
		//don't block on receiving the opponent
		receiverOpp.receive(false);
		
		// Make sure we have a radio
		bool first = true;
		if (!radio)
		{
			while (!radio)
			{
				try
				{
					radio = new Radio(n);
				} catch (exception &ex)
				{
					if (first)
					{
						fprintf(stderr, "%s\n", ex.what());
						fprintf(stderr, "Waiting for the base station to be connected...\n");
						first = false;
					}
					
					sleep(1);
				}
			}
			fprintf(stderr, "\nRadio connected\n");
			
			// Drop this forward packet because it's probably really old
			continue;
		}
		
		//FIXME - Switch between teams for reverse channel
		int reverse_board_id = txPacket.reverse_board_id;
		
		// Build a forward packet
		forward_packet[0] = (sequence << 4) | reverse_board_id;
		forward_packet[1] = 0x0f;
		forward_packet[2] = 0x00;
		
		int offset = 3;
		int kick_id = -1;
		int self_bots = 0;
		uint8_t kick_strength = 0;
		for (int robot_id = 0; robot_id < 5; ++robot_id)
		{
			const Packet::RadioTx::Robot &robot = txPacket.robots[robot_id];
			int board_id = robot.board_id;
			
			int8_t m0, m1, m2, m3;
			uint8_t kick, roller;
			
			if (robot.valid)
			{
				self_bots++;
				
				m1 = -robot.motors[0];
				m2 = -robot.motors[1];
				m3 = -robot.motors[2];
				m0 = -robot.motors[3];
				kick = robot.kick;
				
				if (robot.roller > 0)
				{
					roller = robot.roller * 2;
				} else {
					roller = 0;
				}
			} else {
				board_id = 15;
				m0 = m1 = m2 = m3 = 0;
				kick = 0;
				roller = 0;
			}
			
			if (kick)
			{
				kick_id = board_id;
				kick_strength = kick;
			}
			
			forward_packet[offset++] = m0;
			forward_packet[offset++] = m1;
			forward_packet[offset++] = m2;
			forward_packet[offset++] = m3;
			forward_packet[offset++] = (roller & 0xf0) | (board_id & 0x0f);
		}
		
		if (useOpp)
		{	
			offset = 3 + self_bots * 5;
			for (int robot_id = 0; robot_id < 5 - self_bots; ++robot_id)
			{
				const Packet::RadioTx::Robot &robot = oppTxPacket.robots[robot_id];
				int board_id = robot.board_id;
				
				int8_t m0, m1, m2, m3;
				uint8_t kick, roller;
				
				if (robot.valid)
				{
					m1 = -robot.motors[0];
					m2 = -robot.motors[1];
					m3 = -robot.motors[2];
					m0 = -robot.motors[3];
					kick = robot.kick;
					
					if (robot.roller > 0)
					{
						roller = robot.roller * 2;
					} else {
						roller = 0;
					}
				} else {
					board_id = 15;
					m0 = m1 = m2 = m3 = 0;
					kick = 0;
					roller = 0;
				}
				
				if (kick)
				{
					kick_id = board_id;
					kick_strength = kick;
				}
				
				forward_packet[offset++] = m0;
				forward_packet[offset++] = m1;
				forward_packet[offset++] = m2;
				forward_packet[offset++] = m3;
				forward_packet[offset++] = (roller & 0xf0) | (board_id & 0x0f);
			}
		}

		// ID of kicking robot and kick strength
		if (kick_id >= 0)
		{
			forward_packet[1] = kick_id;
			forward_packet[2] = kick_strength;
		}
		
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
			read_ok = radio->read_packet(reverse_packet, sizeof(reverse_packet), 1);
			rx_time = Utils::timestamp();
		} catch (exception &ex)
		{
			fprintf(stderr, "%s\n", ex.what());
			delete radio;
			radio = 0;
		}
		
		sequence = (sequence + 1) & 15;
		
		// Check for a reverse packet
		if (read_ok)
		{
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
			
			Packet::RadioRx rxPacket;
			int board_id = reverse_packet[0] & 0x0f;
			
			rxPacket.timestamp = rx_time;
			rxPacket.board_id = board_id;
			rxPacket.rssi = (int8_t)reverse_packet[1] / 2.0;
			rxPacket.battery = reverse_packet[3] * 3.3 / 256.0 * 5.0;
			rxPacket.ball = reverse_packet[5] & (1 << 5);
			rxPacket.charged = reverse_packet[4] & 1;
			
			for (int i = 0; i < 5; ++i)
			{
				rxPacket.motorFault[i] = reverse_packet[5] & (1 << i);
			}
			
			for (int i = 0; i < 4; ++i)
			{
				rxPacket.encoders[i] = reverse_packet[6 + i];
				rxPacket.encoders[i] |= ((reverse_packet[10] >> (i * 2)) & 3) << 8;
			}
			
			sender.send(rxPacket);
		}
		
		if (printed)
		{
			printf("\n");
		}
	}

	return 0;
}
