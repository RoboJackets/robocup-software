#pragma once

#include <stdint.h>
#include <libusb.h>
#include <QMutex>

#include "Radio.hpp"

//FIXME - This needs to go somewhere common to this code, the robot firmware, and the base station test code.
const unsigned int Forward_Size = 55;
const unsigned int Reverse_Size = 7;

class USBRadio: public Radio
{
public:
	// n identifies which base station to use.
	USBRadio();
	~USBRadio();

	virtual bool isOpen() const;
	virtual void send(Packet::RadioTx &packet);
	virtual void receive();
	
	virtual void channel(int n);
	
protected:
	libusb_context *_usb_context;
	libusb_device_handle *_device;
	
	// These transfers are used to receive packets.
	// Try increasing this constant for larger RX packet throughput.
	static const int NumRXTransfers = 4;
	libusb_transfer *_rxTransfers[NumRXTransfers];
	uint8_t _rxBuffers[NumRXTransfers][Reverse_Size + 2];
	
	QMutex _mutex;
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
