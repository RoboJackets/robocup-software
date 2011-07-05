//FIXME - Something hangs if PKTCTRL0==4 (fixed length packets) when variable-length packets are in use.

#include <stdio.h>
#include <stdexcept>
#include <boost/format.hpp>
#include <boost/foreach.hpp>

#include <QMutexLocker>

#include <Utils.hpp>
#include "USBRadio.hpp"
#include "cc1101.h"
#include "radio_config.h"

using namespace std;
using namespace boost;
using namespace Packet;

// Timeout for control transfers, in milliseconds
static const int Control_Timeout = 1000;

USBRadio::USBRadio()
{
	_sequence = 0;
	_printedError = false;
	_device = 0;
	_usb_context = 0;
	libusb_init(&_usb_context);
	
	for (int i = 0; i < NumRXTransfers; ++i)
	{
		_rxTransfers[i] = libusb_alloc_transfer(0);
	}
}

USBRadio::~USBRadio()
{
	if (_device)
	{
		libusb_close(_device);
	}
	
	for (int i = 0; i < NumRXTransfers; ++i)
	{
		libusb_free_transfer(_rxTransfers[i]);
	}
	
	libusb_exit(_usb_context);
}

bool USBRadio::open()
{
	libusb_device **devices = 0;
	ssize_t numDevices = libusb_get_device_list(_usb_context, &devices);
	
	if (numDevices < 0)
	{
		fprintf(stderr, "libusb_get_device_list failed\n");
		return false;
	}
	
	int numRadios = 0;
	for (int i = 0; i < numDevices; ++i)
	{
		struct libusb_device_descriptor desc;
		int err = libusb_get_device_descriptor(devices[i], &desc);
		if (err == 0 && desc.idVendor == 0x3141 && desc.idProduct == 0x0004)
		{
			++numRadios;
			int err = libusb_open(devices[i], &_device);
			if (err == 0)
			{
				break;
			}
		}
	}
	
	libusb_free_device_list(devices, 1);
	
	if (!numRadios)
	{
		if (!_printedError)
		{
			fprintf(stderr, "USBRadio: No radio is connected\n");
			_printedError = true;
		}
		return false;
	}
	
	if (!_device)
	{
		if (!_printedError)
		{
			fprintf(stderr, "USBRadio: All radios are in use\n");
			_printedError = true;
		}
		return false;
	}
	
	if (libusb_set_configuration(_device, 1))
	{
		if (!_printedError)
		{
			fprintf(stderr, "USBRadio: Can't set configuration\n");
			_printedError = true;
		}
		return false;
	}
	
	if (libusb_claim_interface(_device, 0))
	{
		if (!_printedError)
		{
			fprintf(stderr, "USBRadio: Can't claim interface\n");
			_printedError = true;
		}
		return false;
	}
	
	configure();
	
	// Start the receive transfers
	for (int i = 0; i < NumRXTransfers; ++i)
	{
		libusb_fill_bulk_transfer(_rxTransfers[i], _device, LIBUSB_ENDPOINT_IN | 2, _rxBuffers[i], Reverse_Size + 2, rxCompleted, this, 0);
		libusb_submit_transfer(_rxTransfers[i]);
	}
	
	_printedError = false;
	
	return true;
}

void USBRadio::rxCompleted(libusb_transfer* transfer)
{
	USBRadio *radio = (USBRadio *)transfer->user_data;
	
	if (transfer->actual_length == Reverse_Size + 2)
	{
		// Parse the packet and add to the list of RadioRx's
		radio->handleRxData(transfer->buffer);
	}
	
	// Restart the transfer
	libusb_submit_transfer(transfer);
}

void USBRadio::command(uint8_t cmd)
{
	if (libusb_control_transfer(_device, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR, 2, 0, cmd, 0, 0, Control_Timeout))
	{
		throw runtime_error("USBRadio::command control write failed");
	}
}

void USBRadio::write(uint8_t reg, uint8_t value)
{
	if (libusb_control_transfer(_device, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR, 1, value, reg, 0, 0, Control_Timeout))
	{
		throw runtime_error("USBRadio::write control write failed");
	}
}

uint8_t USBRadio::read(uint8_t reg)
{
	uint8_t value = 0;
	if (libusb_control_transfer(_device, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR, 3, 0, reg, &value, 1, Control_Timeout))
	{
		throw runtime_error("USBRadio::read control write failed");
	}
	
	return value;
}

void USBRadio::configure()
{
	auto_calibrate(false);
	
	command(SIDLE);
	command(SFTX);
	command(SFRX);
	
	// Write configuration.
	// This is mainly for frequency, bit rate, and packetization.
	for (unsigned int i = 0; i < sizeof(cc1101_regs); i += 2)
	{
		write(cc1101_regs[i], cc1101_regs[i + 1]);
	}

	write(CHANNR, _channel);

	auto_calibrate(true);
}

void USBRadio::auto_calibrate(bool enable)
{
	int flag = enable ? 1 : 0;
	assert(libusb_control_transfer(_device, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR, 4, flag, 0, 0, 0, Control_Timeout) == 0);
}

bool USBRadio::isOpen() const
{
	return _device;
}

void USBRadio::send(Packet::RadioTx& packet)
{
	QMutexLocker lock(&_mutex);
	if (!_device)
	{
		if (!open())
		{
			return;
		}
	}
	
	uint8_t forward_packet[Forward_Size];
	
	// Build a forward packet
	forward_packet[0] = _sequence;
	packet.set_sequence(_sequence);
	
	int offset = 1;
	int self_bots = 0;
	int robot_id;
	for (robot_id = 0; robot_id < 5 && robot_id < packet.robots_size(); ++robot_id)
	{
		//FIXME - Read from both channels and merge
		const RadioTx::Robot &robot = packet.robots(robot_id);
		int robot_id = robot.robot_id();
		
		int8_t m0, m1, m2, m3;
		uint8_t kick, dribbler;
		
		self_bots++;
		
		m1 = -robot.motors(0);
		m2 = -robot.motors(1);
		m3 = -robot.motors(2);
		m0 = -robot.motors(3);
		kick = robot.kick();
		
		if (robot.dribbler() > 0)
		{
			dribbler = robot.dribbler() * 2;
		} else {
			dribbler = 0;
		}
		
		forward_packet[offset++] = m0;
		forward_packet[offset++] = m1;
		forward_packet[offset++] = m2;
		forward_packet[offset++] = m3;
		forward_packet[offset++] = (dribbler & 0xf0) | (robot_id & 0x0f);
		forward_packet[offset++] = kick;
		forward_packet[offset++] = robot.use_chipper() ? 1 : 0;
	}
	
	// Unused slots
	for (; robot_id < 5; ++robot_id)
	{
		forward_packet[offset++] = 0;
		forward_packet[offset++] = 0;
		forward_packet[offset++] = 0;
		forward_packet[offset++] = 0;
		forward_packet[offset++] = 0x0f;
		forward_packet[offset++] = 0;
		forward_packet[offset++] = 0;
	}
	
	// Send the forward packet
	int sent = 0;
	if (libusb_bulk_transfer(_device, LIBUSB_ENDPOINT_OUT | 1, forward_packet, sizeof(forward_packet), &sent, Control_Timeout) || sent != sizeof(forward_packet))
	{
		fprintf(stderr, "USBRadio: Bulk write failed\n");
		libusb_close(_device);
		_device = 0;
	}
	
	_sequence = (_sequence + 1) & 7;
}

void USBRadio::receive()
{
	QMutexLocker lock(&_mutex);
	
	if (!_device)
	{
		if (!open())
		{
			return;
		}
	}
	
	// Handle USB events.  This will call callbacks.
	struct timeval tv = {0, 0};
	libusb_handle_events_timeout(_usb_context, &tv);
}

void USBRadio::handleRxData(uint8_t *buf)
{
	uint64_t rx_time = Utils::timestamp();
	
	_reversePackets.push_back(RadioRx());
	RadioRx &packet = _reversePackets.back();
	
	packet.set_timestamp(rx_time);
	packet.set_sequence((buf[0] >> 4) & 7);
	packet.set_robot_id(buf[0] & 0x0f);
	packet.set_rssi((int8_t)buf[1] / 2.0 - 74);
	packet.set_battery(buf[2] / 10.0f);
	packet.set_kicker_status(buf[3]);
	
	// Drive motor status
	for (int i = 0; i < 4; ++i)
	{
		packet.add_motor_status(MotorStatus((buf[4] >> (i * 2)) & 3));
	}
	
	// Dribbler status
	packet.add_motor_status(MotorStatus(buf[5] & 3));
	
	// Hardware version
	if (buf[5] & 4)
	{
		packet.set_hardware_version(RJ2008);
	} else {
		packet.set_hardware_version(RJ2011);
	}

	packet.set_ball_sense_status(BallSenseStatus((buf[5] >> 2) & 3));
	
	// Encoders
	for (int i = 0; i < 4; ++i)
	{
		int high = (buf[10] >> (i * 2)) & 3;
		int16_t value = buf[6 + i] | (high << 8);
		if (high & 2)
		{
			value |= 0xfc00;
		}
		packet.add_encoders(value);
	}
	
	packet.set_kicker_voltage(buf[11]);
}

void USBRadio::channel(int n)
{
	QMutexLocker lock(&_mutex);
	
	if (_device)
	{
		auto_calibrate(false);
		
		write(CHANNR, n);
		
		command(SIDLE);
		command(SRX);
		
		auto_calibrate(true);
	}
	
	Radio::channel(n);
}
