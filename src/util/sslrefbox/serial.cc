/*========================================================================
    Serial.cc : Class wrapper for Serial I/O on Linux
  ------------------------------------------------------------------------
    Copyright (C) 1999-2002  James R. Bruce
    School of Computer Science, Carnegie Mellon University
  ------------------------------------------------------------------------
    This software is distributed under the GNU General Public License,
    version 2.  If you do not have a copy of this licence, visit
    www.gnu.org, or write: Free Software Foundation, 59 Temple Place,
    Suite 330 Boston, MA 02111-1307 USA.  This program is distributed
    in the hope that it will be useful, but WITHOUT ANY WARRANTY,
    including MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  ========================================================================*/

#ifndef WIN32
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#else
# include <cstdlib>
# include <cstdio>
#endif                             

#include "serial.h"

// controls
#define DEBUG

/**************************** TYPES ******************************************/

// #define RADIO_DEVICE "/dev/ttyS0"
#ifndef WIN32

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


/**************************** CODE *******************************************/


/*
 * baud_rate_to_flag -
 *
 * convert from raw baud rate to teh 
 * appropriate baud flag
 */
speed_t baud_rate_to_flag(unsigned int speed)
{
  int i;

  for (i = 0; i < BAUD_RATES; i++){
    if (baud_rates[i][1] == speed) 
    	return(baud_rates[i][0]);
  }

  return(0);
}


/*
 * baud_flag_to_rate -
 *
 * This function converts from the speed s to
 * the baud flag 
 */
int baud_flag_to_rate(speed_t s)
{
  int i;

  for(i = 0; i < BAUD_RATES; i++){
    if (baud_rates[i][0] == s) 
    	return(baud_rates[i][1]);
  }

  return(0);
}

#endif
/**************************** SERIAL CLASS ***********************************/


/*
 * open -
 *
 * This function opens the serial port device and configures it
 * to operate at the specified baud rate and parameters
 *
 * rETURN VALUE: TRUE on success, FALSE on failure
 */
bool Serial::Open(char *device,int baud)
{
#ifdef WIN32
	DCB dcb;
	hCom = CreateFile( device,
		GENERIC_READ | GENERIC_WRITE,
		0,    // exclusive access 
		NULL, // no security attributes 
		OPEN_EXISTING, 0, NULL);

	if (hCom == INVALID_HANDLE_VALUE)
	{ 
		fprintf(stderr,"ERROR: Can't open comm port.\n");
		return false;
	}

	if (!GetCommState(hCom, &dcb))
	{
		fprintf(stderr,"ERROR: Can't get CommState.\n");
		Close();
		return false;
	}
	
 	switch (baud)
 	{
 		case 9600:
			dcb.BaudRate = CBR_9600;
			break;
 		case 19200:
			dcb.BaudRate = CBR_19200;
			break;
 		case 110:
			dcb.BaudRate = CBR_110;
			break;
 		case 300:
			dcb.BaudRate = CBR_300;
			break;
 		case 38400:
			dcb.BaudRate = CBR_38400;
			break;
 		case 600:
			dcb.BaudRate = CBR_600;
			break;
 		case 56000:
			dcb.BaudRate = CBR_56000;
			break;
 		case 1200:
			dcb.BaudRate = CBR_1200;
			break;
 		case 57600:
			dcb.BaudRate = CBR_57600;
			break;
 		case 2400:
			dcb.BaudRate = CBR_2400;
			break;
 		case 115200:
			dcb.BaudRate = CBR_115200;
			break;
 		case 4800:
			dcb.BaudRate = CBR_4800;
			break;
 		case 128000:
			dcb.BaudRate = CBR_128000;
			break;
 		case 256000:
			dcb.BaudRate = CBR_256000;
			break;
 		case 14400:
			dcb.BaudRate = CBR_14400;
			break;
		default:
			fprintf(stderr,"ERROR: Baut rate currently not supported.\n");
			Close();
			return false;
	}			
			
	dcb.ByteSize =8;
	dcb.fParity = FALSE;
	dcb.fDsrSensitivity = FALSE;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	dcb.fOutxCtsFlow =FALSE;
	dcb.fOutxDsrFlow = FALSE;
	dcb.fOutX = FALSE;
	dcb.fInX = FALSE;
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;

	if(!SetCommState(hCom,&dcb))
	{
		fprintf(stderr,"ERROR: Can't set CommState.\n");
		Close();
		return false;
	}
	return true; 
#else
	struct termios tio;
	speed_t bf;
	int baud_in,baud_out;

	// open device
	fd = ::open(device,O_RDWR|O_SYNC);
	if (fd < 0) 
		return(false);

	/* set control attributes -
	 * binary I/O, no flow control
	 */
	tcgetattr(fd,&tio);
	cfmakeraw(&tio);
	tio.c_cflag &= ~(CRTSCTS|IXON|IXOFF);

	// set serial port speed
	bf = baud_rate_to_flag(baud);
	cfsetispeed(&tio,bf);
	cfsetospeed(&tio,bf);
	tcsetattr(fd,TCSANOW,&tio);

	// test what actually got set
	memset(&tio,0,sizeof(tio));
	tcgetattr(fd,&tio);
	baud_in  = baud_flag_to_rate(cfgetispeed(&tio));
	baud_out = baud_flag_to_rate(cfgetospeed(&tio));

	return((baud_in==baud) && (baud_out==baud));
#endif	
}


/*
 * close -
 *
 * this function closes the serial port module 
 */
void Serial::Close(void)
{
#ifdef WIN32	
	if(!CloseHandle(hCom))
		fprintf(stderr,"ERROR: Can't close comm port.\n");
	hCom=INVALID_HANDLE_VALUE;
#else
	if(fd){
		::close(fd);
		fd = 0;
	}
#endif
}

/*
 * Read -
 *
 * function reads bytes from the serial port
 */
#ifdef WIN32 
int Serial::Read(void *buf,int size)
{
	DWORD bytesTransfered;
	if(!ReadFile(hCom, (char*)buf,size, &bytesTransfered, NULL))
		fprintf(stderr,"ERROR: Can't read from comm port.\n");
	return bytesTransfered;	
}
#endif

/*
 * Write -
 *
 * Function writes bytes to the serial port 
 */
#ifdef WIN32
int Serial::Write(void *buf,int size)
{
	DWORD bytesTransfered;
	if(!WriteFile(hCom, (char*)buf, size, &bytesTransfered, NULL ))
		fprintf(stderr,"ERROR: Can't write to comm port.\n");
	return bytesTransfered;	
}
#endif

/*
 * Writebyte -
 *
 * Function writes a signel byte to the serial port 
 */
int Serial::WriteByte(char b)
{
#ifdef DEBUG
	printf("writing %c\n", b);
#endif
#ifdef WIN32
	return Write(&b,1);
#else
	return (::write(fd, &b, 1));
#endif
}


/*
 * ReadByte -
 *
 * function reads a single byte from the serial port
 */
int Serial::ReadByte(void)
{
#ifdef WIN32
	char b;
	return Read(&b,1);
	return b;
#else
	char	data;
	int		ret_val;
	
	if ((ret_val = ::read(fd, &data, 1)) < 0)
		return (ret_val);
	return (data);
#endif
}
