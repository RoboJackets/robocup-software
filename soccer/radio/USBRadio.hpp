#pragma once

#include <stdint.h>

#include <libusb.h>

#include "Radio.hpp"

//FIXME - This needs to go somewhere common to this code, the robot firmware, and the base station test code.
const unsigned int Forward_Size = 31;
const unsigned int Reverse_Size = 11;

class USBRadio: public Radio
{
public:
	// n identifies which base station to use.
	USBRadio();
	~USBRadio();

	virtual bool isOpen() const;
	virtual void send(const Packet::RadioTx &packet);
	virtual void receive();
	
protected:
	libusb_context *_usb_context;
	libusb_device_handle *_device;
	
	// Thes transfers is used to receive packets
	static const int NumRXTransfers = 2;
	libusb_transfer *_rxTransfers[NumRXTransfers];
	uint8_t _rxBuffers[NumRXTransfers][Reverse_Size + 2];
	
	int _sequence;
	bool _printedError;
	
	static void rxCompleted(struct libusb_transfer *transfer);
	void handleRxData(uint8_t *buf);
	
	bool open();
	
	// Low level operations
	void command(uint8_t cmd);
	void write(uint8_t reg, uint8_t value);
	uint8_t read(uint8_t reg);
	
	// Turns on/off automatic calibration when there is no traffic
	void auto_calibrate(bool enable);
	
	// Configures the base station firmware and radio
	void configure();
};
