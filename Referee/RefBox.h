#ifndef REFBOX_H_
#define REFBOX_H_

/*
 Address: 224.5.23.1
 Port: 10001

 One UDP Packet from Ref
 +---------+---------+---------+---------+-------------------+
 | Command | Counter | Goals bl| Goals ye| time left (16 Bit)|
 +---------+---------+---------+---------+-------------------+

 16 bit timeleft is in network byte order
 */

typedef struct
{
		char cmd;
		unsigned char counter;
		unsigned char goalsBlue;
		unsigned char goalsYellow;
		unsigned short time;
} RefPacket;

#endif /* REFBOX_H_ */
