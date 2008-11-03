#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <unistd.h>

class Serial
{
  /** file descriptor used to open serial port device */
  int fd;

public:
  Serial();
  ~Serial(); 


  /** prototypes */
  bool Open(char *device,int baud);
  void Close(void);

  
  /** This function reads bytes from the serial port */
  int Read(void *buf,int size){
	  return(::read(fd,buf,size));
	  }
  
  /** This function writes bytes to the serial port */
  int Write(void *buf,int size){
	  return(::write(fd,buf,size));
	  }
  
  /** This function writes a single byte to the serial port */  
  int WriteByte(char b);
  
  /** This function reads a single byte from the serial port */
  int ReadByte(void);
  	
};

#endif /*SERIAL_HPP_*/
