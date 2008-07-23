#include "Radio.hpp"
#include "Serial.hpp"

#include <stdexcept>
#include <stdint.h>

using namespace std;

Radio::Radio(const char *device): _serial(device, 38400)
{
	_command_mode = false;
	command_mode(false);
}

void Radio::command_mode(bool flag)
{
	_serial.dtr(!flag);
	
	if (!_command_mode && flag)
	{
		for (int i = 0; i < 2; ++i)
		{
			uint8_t ch = _serial.read();
			printf("Command mode acknowledge %d:%02x\n", i, ch);
			if (ch != 0xff)
			{
				i = -1;
			}
		}
	}
	
	_command_mode = flag;
}

int Radio::set_channels(int rx, int tx)
{
	uint8_t buf[2] = {rx, tx};
	_serial.write(buf, sizeof(buf));
	
	uint8_t rssi[2];
	_serial.read(rssi, sizeof(rssi));
	
	return rssi[1] * 256 + rssi[0];
}
