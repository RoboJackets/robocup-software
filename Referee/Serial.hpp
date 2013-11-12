#pragma once

#include <cstdlib>
#include <cstdio>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>

class Serial
{
		/// methods ///
	public:
				
		Serial();
		~Serial(); 
  
		/** function opens the serial port device and configures it */
		bool open(char *device,int baud);
  
		/** function closes the serial port module */
		void close(void);
		
		/**  * This function converts from the speed s to the baud flag */
		int baudFlagToRate(speed_t s);
			
		/** convert from raw baud rate to teh appropriate baud flag */
		speed_t baudRateToFlag(unsigned int speed);
  
		/** This function writes a single byte to the serial port */  
		int writeByte(char b);
  
		/** This function reads a single byte from the serial port */
		int readByte(void);
		
  
		/// members ///
	private:
		/** file descriptor used to open serial port device */
		int _fileDescriptor;
  	
};
