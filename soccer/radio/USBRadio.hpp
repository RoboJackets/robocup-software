#pragma once

#include <stdint.h>
#include <libusb.h>
#include <QMutex>

#include "Radio.hpp"

//FIXME - This needs to go somewhere common to this code, the robot firmware, and the base station test code.
const unsigned int Forward_Size = 55;
const unsigned int Reverse_Size = 7;

/**
 * @brief Radio IO with real robots
 * 
 * @details This class provides us the ability to communicate with real robots using our own radio protocol.
 * The radio sends one large packet to all of the robots at once that contains the data in each
 * robot's radioTx packet.  Note that it isn't sent in protobuf format though, it's sent straight-up
 * data to avoid the overhead of protobuf.  Robots respond individually in order of their shell
 * numbers in a set time slot.  The bot with the lowest shell number replies in the first time slot,
 * and so on.  This ensures that robots don't jam each other's communication.
 */
class USBRadio: public Radio
{
public:
	// n identifies which base station to use.
    USBRadio();
    ~USBRadio();

	virtual bool isOpen() const override;
	virtual void send(Packet::RadioTx &packet) override;
	virtual void receive() override;
	
	virtual void channel(int n) override;
    void switchTeam(bool) override { }
	
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
