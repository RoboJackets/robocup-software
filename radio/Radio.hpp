#pragma once

#include <stdint.h>

//FIXME - This needs to go somewhere common to this code, the robot firmware, and the base station test code.
const unsigned int Forward_Size = 31;
const unsigned int Reverse_Size = 11;

class USB_Device;

class Radio
{
public:
	// n identifies which base station to use.
	Radio(int n = 0);
	~Radio();

	USB_Device *device() const
	{
		return _device;
	}

	void write_packet(const void *data, unsigned int size);
	bool read_packet(void *data, unsigned int size, int timeout = 0);
	
protected:
	USB_Device *_device;
	
	// Low level operations
	void command(uint8_t cmd);
	void write(uint8_t reg, uint8_t value);
	uint8_t read(uint8_t reg);
	
	// Sets the size of reverse packets.
	// The base station firmware uses this when in receive mode.
	void reverse_size(int size);
	
	// Turns on/off automatic calibration when there is no traffic
	void auto_calibrate(bool enable);
	
	// Configures the base station firmware and radio
	void configure();
};
