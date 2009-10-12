// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <assert.h>

#include <map>

#include <Network/Network.hpp>
//#include <Network/Receiver.hpp>
#include <Network/PacketReceiver.hpp>
#include <Network/Sender.hpp>
#include <RadioTx.hpp>
#include <RadioRx.hpp>
#include <Utils.hpp>

#include "Radio.hpp"

using namespace std;

Radio *radio;
Team team = UnknownTeam;

pthread_mutex_t mapping_mutex;
typedef map<int, int> Robot_Map;
Robot_Map board_to_robot;

bool useOpp;

Packet::RadioTx txPacket;
Packet::RadioTx oppTxPacket;

void usage(const char* prog)
{
	fprintf(stderr, "usage: %s [-n i] <-y|-b>\n", prog);
	fprintf(stderr, "\t-y: run as the yellow team\n");
	fprintf(stderr, "\t-b: run as the blue team\n");
	fprintf(stderr, "\t-n: use base station i (0 is the first base station detected by libusb)\n");
	fprintf(stderr, "\t-o: run an opponent team as well (note: radio only handles up to 5 robots)\n");
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

	radio = new Radio(n);
	
	if (team == UnknownTeam)
	{
		fprintf(stderr, "Error: No team specified\n");
		usage(argv[0]);
	}
	
	Network::Sender sender(Network::Address, Network::addTeamOffset(team, Network::RadioRx));
	Packet::RadioRx rxPacket;
	
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
	int reverse_robot_id = 0;
	
	uint64_t lastTime = Utils::timestamp();
	
	while (true)
	{
		//clear the in coming packet
		txPacket = Packet::RadioTx();
		
		//block for self
		receiverSelf.receive(true);
		
		//don't block on receiving the opponent
		receiverOpp.receive(false);
		
		// Update the mapping for the reverse thread
		pthread_mutex_lock(&mapping_mutex);
		board_to_robot.clear();
		for (int robot_id = 0; robot_id < 5; ++robot_id)
		{
			int board_id = txPacket.robots[robot_id].board_id;
			board_to_robot[board_id] = robot_id;
		}
		pthread_mutex_unlock(&mapping_mutex);
		
		// Find the next robot to send a reverse packet
		int start = reverse_robot_id;
		do
		{
			if (reverse_robot_id == 4)
			{
				reverse_robot_id = 0;
			} else {
				++reverse_robot_id;
			}
		} while (reverse_robot_id != start && !txPacket.robots[reverse_robot_id].valid);
		
		int reverse_board_id = 15;
		
		if (txPacket.robots[reverse_robot_id].valid)
		{
			reverse_board_id = txPacket.robots[reverse_robot_id].board_id;
		}
		
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
		
		uint64_t t1 = Utils::timestamp();
		uint64_t dt = t1 - lastTime;
		lastTime = t1;
#if 1
		printf("%3ld.%03ldms ", dt / 1000, dt % 1000);
		
		for (unsigned int i = 0; i < Forward_Size; ++i)
		{
			printf("%02x ", forward_packet[i]);
		}
		printf("\n");
#endif
		
		// Send the forward packet
		radio->write_packet(forward_packet, Forward_Size);
		
		sequence = (sequence + 1) & 15;
		
		// Check for a reverse packet
		if (radio->read_packet(reverse_packet, sizeof(reverse_packet), 1))
		{
#if 1
			printf("rev");
			for (unsigned int i = 0; i < Reverse_Size; ++i)
			{
				printf(" %02x", reverse_packet[i]);
			}
			printf("\n");
#endif
			
			int board_id = reverse_packet[0] & 0x0f;
			
			// Board to robot mapping
			pthread_mutex_lock(&mapping_mutex);
			Robot_Map::const_iterator i = board_to_robot.find(board_id);
			if (i == board_to_robot.end())
			{
				// We don't know about this board
				pthread_mutex_unlock(&mapping_mutex);
				continue;
			}
			int robot_id = board_to_robot[board_id];
			pthread_mutex_unlock(&mapping_mutex);
			
			Packet::RadioRx::Robot &robot = rxPacket.robots[robot_id];
			
			// Set the updated flag for only this robot
			for (int i = 0; i < 5; ++i)
			{
				rxPacket.robots[i].updated = false;
			}
			robot.updated = true;
			robot.valid = true;
			
			for (int i = 0; i < 5; ++i)
			{
				robot.motorFault[i] = reverse_packet[5] & (1 << i);
			}
			
			robot.rssi = (int8_t)reverse_packet[1] / 2.0;
			robot.battery = reverse_packet[3] * 3.3 / 256.0 * 5.0;
			robot.ball = reverse_packet[5] & (1 << 5);
			robot.charged = reverse_packet[4] & 1;
			
			for (int i = 0; i < 5; ++i)
			{
				robot.encoders[i] = reverse_packet[6 + i];
			}
			
			sender.send(rxPacket);
		}
	}

	return 0;
}
