#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <errno.h>
#include <stdexcept>

#include "Serial.hpp"

using namespace std;

Serial::Serial(const char *device, int baud)
{
	struct termios attr;

	int baud_flag;
	switch (baud)
	{
		case 9600:
			baud_flag = B9600;
			break;
		case 19200:
			baud_flag = B19200;
			break;
		case 38400:
			baud_flag = B38400;
			break;
		case 57600:
			baud_flag = B57600;
			break;
		case 115200:
			baud_flag = B115200;
			break;
		default:
			throw range_error("Bad baud rate");
	}

	/* Mac OS X: O_NONBLOCK is required because (in some cases?) the open
	 * will block until the hardware flow control lines are set correctly,
	 * and if this never happens the program hangs and becomes unkillable.
	 */
	_handle = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (_handle < 0)
	{
		throw runtime_error(strerror(errno));
	}

	tcgetattr(_handle, &attr);
	cfsetispeed(&attr, baud_flag);
	cfsetospeed(&attr, baud_flag);
	cfmakeraw(&attr);
	attr.c_cflag |= CLOCAL | CREAD;
	attr.c_iflag &= ~IXOFF & ~IXANY;
	tcsetattr(_handle, TCSANOW, &attr);

	/* It's safe to block now that flow control has been turned off */
	fcntl(_handle, F_SETFL, 0);
}

Serial::~Serial()
{
	if (_handle >= 0)
	{
		close(_handle);
	}
}

void Serial::write(const void *buf, size_t len)
{
	::write(_handle, buf, len);
	fsync(_handle);
}

unsigned char Serial::read()
{
	unsigned char byte;

	::read(_handle, &byte, 1);

	return byte;
}

void Serial::dtr(bool flag)
{
    int flags = 0;
    ioctl(_handle, TIOCMGET, &flags);

    if (flag)
    {
        flags |= TIOCM_DTR;
    } else {
        flags &= ~TIOCM_DTR;
    }

    ioctl(_handle, TIOCMSET, &flags);
}

void Serial::read(void *buf, size_t len)
{
	while (len)
	{
		int n = ::read(_handle, buf, len);
		if (n < 0)
		{
			memset(buf, 0, len);
			return;
		}

		len -= n;
		buf = (char *)buf + n;
	}
}
