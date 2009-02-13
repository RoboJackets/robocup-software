#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#include <Network/Network.hpp>
#include <Network/Receiver.hpp>
#include <Network/Sender.hpp>
#include <RadioTx.hpp>
#include <RadioRx.hpp>

#include "Radio.hpp"

Radio *radio;
volatile bool reverse_running;
pthread_t reverse_thread;
Team team = UnknownTeam;

void usage(const char* prog)
{
	fprintf(stderr, "usage: %s [-n i] <-y|-b>\n", prog);
	fprintf(stderr, "\t-y: run as the yellow team\n");
	fprintf(stderr, "\t-b: run as the blue team\n");
    fprintf(stderr, "\t-n: use base station i (0 is the first base station detected by libusb)\n");
    exit(1);
}

void *reverse_main(void *arg)
{
    reverse_running = false;
    
    //data coming in from robots goes out through the sender
    Network::Sender sender(Network::Address, Network::addTeamOffset(team, Network::RadioRx));

    uint8_t reverse_packet[Reverse_Size];
    
    Packet::RadioRx rxPacket;
    
    while (reverse_running)
    {
        if (radio->read_packet(reverse_packet, Reverse_Size, 250))
        {
            int board_id = reverse_packet[0] & 0x0f;
            
            //FIXME - Board to robot mapping
            int robot_id = board_id;
            
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
            
            robot.rssi = (int8_t)reverse_packet[1] * 2.0f;
            robot.tuning = reverse_packet[2];
            robot.capacitor = reverse_packet[3];    //FIXME - Conversion
            robot.battery = reverse_packet[4];      //FIXME - Conversion
            
            for (int i = 0; i < 5; ++i)
            {
                robot.encoders[i] = reverse_packet[6 + i];
            }
            
            sender.send(rxPacket);
        }
    }
    
    return 0;
}

void reverse_start()
{
    pthread_create(&reverse_thread, 0, reverse_main, 0);
}

void reverse_stop()
{
    reverse_running = false;
    pthread_join(reverse_thread, 0);
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
	}

    radio = new Radio(n);
    
	if (team == UnknownTeam)
	{
		fprintf(stderr, "Error: No team specified\n");
		usage(argv[0]);
	}
	
	//receiver for incoming radio data from the control program
	Network::Receiver receiver(Network::Address, Network::addTeamOffset(team, Network::RadioTx));
	
    uint8_t forward_packet[Forward_Size];
    int sequence = 0;
    int reverse_board_id = 0;

    while (true)
	{
		//from control program -> radio program
		Packet::RadioTx txPacket;
		receiver.receive(txPacket);
		
        // Build a forward packet
        forward_packet[0] = (sequence << 4) | reverse_board_id;
        forward_packet[1] = 0x07;
        forward_packet[2] = 0x00;
        
        int offset = 3;
        int kick_id = -1;
        uint8_t kick_strength = 0;
        for (int robot_id = 0; robot_id < 5; ++robot_id)
        {
            //FIXME - Robot to board mapping
            int board_id = robot_id;
            
            const Packet::RadioTx::Robot &robot = txPacket.robots[robot_id];
            
            int8_t m0, m1, m2, m3;
            uint8_t kick, roller;
            
            if (robot.valid)
            {
                m0 = robot.motors[0];
                m1 = robot.motors[1];
                m2 = robot.motors[2];
                m3 = robot.motors[3];
                kick = robot.kick;
                roller = robot.roller;
            } else {
                m0 = m1 = m2 = m3 = 0;
                kick = 0;
                roller = 0;
            }
            
            if (kick)
            {
                kick_id = robot_id;
            }
            
            forward_packet[offset++] = m0;
            forward_packet[offset++] = m1;
            forward_packet[offset++] = m2;
            forward_packet[offset++] = m3;
            forward_packet[offset++] = (roller & 0xf0) | (board_id & 0x07);
        }

        // ID of kicking robot and kick strength
        if (kick_id >= 0)
        {
            forward_packet[1] = kick_id;
            forward_packet[2] = kick_strength;
        }
        
        // Send the forward packet
        radio->write_packet(forward_packet, Forward_Size);
        
        sequence = (sequence + 1) & 15;
        reverse_board_id = (reverse_board_id + 1) & 15;
    }
	
	return 0;
}
