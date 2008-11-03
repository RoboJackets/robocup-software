#include <cstdlib>
#include <cstdio>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include "Serial.hpp"

/** controls */
#define DEBUG
#define RADIO_DEVICE "/dev/ttyS0"
#define BAUD_RATES 20
const speed_t baud_rates[BAUD_RATES][2] = {
  {      B0,      0 },
  {     B50,     50 },
  {     B75,     75 },
  {    B110,    110 },
  {    B134,    134 },
  {    B150,    150 },
  {    B200,    200 },
  {    B300,    300 },
  {    B600,    600 },
  {   B1200,   1200 },
  {   B1800,   1800 },
  {   B2400,   2400 },
  {   B4800,   4800 },
  {   B9600,   9600 },
  {  B19200,  19200 },
  {  B38400,  38400 },
  {  B57600,  57600 },
  { B115200, 115200 },
  { B230400, 230400 },
  { B460800, 460800 }
};


/**
 * baud_rate_to_flag -
 *
 * convert from raw baud rate to teh 
 * appropriate baud flag */
speed_t baud_rate_to_flag(unsigned int speed)
{
  int i;

  for (i = 0; i < BAUD_RATES; i++){
    if (baud_rates[i][1] == speed) 
    	return(baud_rates[i][0]);
  }

  return(0);
}


/**
 * baud_flag_to_rate -
 *
 * This function converts from the speed s to
 * the baud flag */
int baud_flag_to_rate(speed_t s)
{
  int i;

  for(i = 0; i < BAUD_RATES; i++){
    if (baud_rates[i][0] == s) 
    	return(baud_rates[i][1]);
  }

  return(0);
}


/**************************** SERIAL CLASS ***********************************/

Serial::Serial(){
	fd=0;
}


Serial::~Serial(){
	   if (fd)
	    {
	    	close(fd);
	    }
}


/** Open - function opens the serial port device and configures it
 *        operate at the specified baud rate and parameters
 * return value: TRUE on success, FALSE on failure */        
bool Serial::Open(char *device,int baud)
{
	struct termios tio;
	speed_t bf;
	int baud_in,baud_out;

	/** open device */
	fd = ::open(device,O_RDWR|O_SYNC);
	if (fd < 0) 
		return(false);

	/** set control attributes -
	 * binary I/O, no flow control */
	tcgetattr(fd,&tio);
	cfmakeraw(&tio);
	tio.c_cflag &= ~(CRTSCTS|IXON|IXOFF);

	/** set serial port speed */
	bf = baud_rate_to_flag(baud);
	cfsetispeed(&tio,bf);
	cfsetospeed(&tio,bf);
	tcsetattr(fd,TCSANOW,&tio);

	/** test what actually got set */
	memset(&tio,0,sizeof(tio));
	tcgetattr(fd,&tio);
	baud_in  = baud_flag_to_rate(cfgetispeed(&tio));
	baud_out = baud_flag_to_rate(cfgetospeed(&tio));

	return((baud_in==baud) && (baud_out==baud));	
}


/** Close - function closes the serial port module */
void Serial::Close(void)
{
	if(fd){
		::close(fd);
		fd = 0;
	}
}


/** WriteByte - function writes a single byte from the serial port */
int Serial::WriteByte(char b)
{
#ifdef DEBUG
	printf("writing %c\n", b);
#endif

	return (::write(fd, &b, 1));
}


/** ReadByte - function reads a single byte from the serial port */
int Serial::ReadByte(void)
{
	char data;
	int ret_val;
	
	if ((ret_val = ::read(fd, &data, 1)) < 0)
	{
		return (ret_val);
	}
	
	return (data);
}
