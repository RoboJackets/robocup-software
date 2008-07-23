#ifndef _SERIAL_HPP_
#define _SERIAL_HPP_

#include <fcntl.h>

class Serial
{
public:
	Serial(const char *path, int baud);
	~Serial();
	
	void write(unsigned char ch);
	unsigned char read();
	
	void write(const void *buf, size_t len);
	void read(void *buf, size_t len);
	
	void dtr(bool flag);
	
	int handle() const { return _handle; }
	
protected:
	int _handle;
};

#endif // _SERIAL_HPP_
