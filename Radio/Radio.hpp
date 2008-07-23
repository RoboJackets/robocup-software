#ifndef _RADIO_HPP_
#define _RADIO_HPP_

#include "Serial.hpp"

class Radio
{
public:
	// Use this as a channel number to turn off the receiver/transmitter.
	static const int Off = 254;
	
	Radio(const char *device);
	
	Serial *serial() { return &_serial; }
	
	void command_mode(bool flag);
	
	// Sets receive and transmit channels and returns RSSI from the previous receive channel.
	int set_channels(int rx, int tx);
	
protected:
	Serial _serial;
	bool _command_mode;
};

#endif // _RADIO_HPP_
