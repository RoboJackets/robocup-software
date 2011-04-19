#pragma once

#include <stdint.h>

#include "Radio.hpp"

//FIXME - This needs to go somewhere common to this code, the robot firmware, and the base station test code.
const unsigned int Forward_Size = 31;
const unsigned int Reverse_Size = 11;

class USB_Device;

class USBRadio: public Radio
{
public:
	// n identifies which base station to use.
	USBRadio();
	~USBRadio();

	USB_Device *device() const
	{
		return _device;
	}
	
	virtual bool isOpen() const;
	virtual void send(const Packet::RadioTx &packet);
	virtual bool receive(Packet::RadioRx *packet);

protected:
	USB_Device *_device;

	int _sequence;
	bool _printedError;
	
	bool open();
	
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
