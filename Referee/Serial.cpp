#include "Serial.hpp"

/** controls */
#define DEBUG
#define RADIO_DEVICE "/dev/ttyS0"
#define BAUD_RATES 20
const speed_t BaudRates[BAUD_RATES][2] = {
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


/// Serial Class ///

Serial::Serial(){
	_fileDescriptor=0;
}

Serial::~Serial(){
	   if (_fileDescriptor)
	    {
		   Serial::close();
	    }
}


/** This function opens the serial port device and configures it
 *  to operate at the specified baud rate and parameters
 *  return value: TRUE on success, FALSE on failure 
 */        
bool Serial::open(char *device,int baud)
{
	struct termios terminalIO;
	speed_t baudSpeed;
	int baudIn;
	int baudOut;

	/** open device */
	_fileDescriptor = ::open(device,O_RDWR|O_SYNC);
	if (_fileDescriptor < 0) 
		return(false);

	/** set control attributes - binary I/O, no flow control */
	tcgetattr(_fileDescriptor,&terminalIO);
	cfmakeraw(&terminalIO);
	terminalIO.c_cflag &= ~(CRTSCTS|IXON|IXOFF);

	/** set serial port speed */
	baudSpeed = baudRateToFlag(baud);
	cfsetispeed(&terminalIO,baudSpeed);
	cfsetospeed(&terminalIO,baudSpeed);
	tcsetattr(_fileDescriptor,TCSANOW,&terminalIO);

	/** test what actually got set */
	memset(&terminalIO,0,sizeof(terminalIO));
	tcgetattr(_fileDescriptor,&terminalIO);
	baudIn  = baudFlagToRate(cfgetispeed(&terminalIO));
	baudOut = baudFlagToRate(cfgetospeed(&terminalIO));

	return((baudIn==baud) && (baudOut==baud));	
}


/** Close - function closes the serial port module */
void Serial::close(void)
{
	if(_fileDescriptor){
		::close(_fileDescriptor);
		_fileDescriptor = 0;
	}
}


/** writeByte - function writes a single byte from the serial port */
int Serial::writeByte(char b)
{
#ifdef DEBUG
	printf("writing %c\n", b);
#endif

	return (::write(_fileDescriptor, &b, 1));
}


/** readByte - function reads a single byte from the serial port */
int Serial::readByte(void)
{
	char data;
	int returnValue;
	
	if ((returnValue = ::read(_fileDescriptor, &data, 1)) < 0)
	{
		return (returnValue);
	}
	
	return (data);
}


 /** This function convert from raw baud rate to the appropriate baud flag */
speed_t Serial::baudRateToFlag(unsigned int speed)
{
  int i;

  for (i = 0; i < BAUD_RATES; i++){
    if (BaudRates[i][1] == speed) 
    	return(BaudRates[i][0]);
  }

  return(0);
}


 /** This function converts from the speed s to the baud flag */
int Serial::baudFlagToRate(speed_t speed)
{
  int i;

  for(i = 0; i < BAUD_RATES; i++){
    if (BaudRates[i][0] == speed) 
    	return(BaudRates[i][1]);
  }

  return(0);
}

