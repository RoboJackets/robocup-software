//FIXME - Port to libusb-1.0.  The API is better.

#include <stdio.h>
#include <stdexcept>
#include <boost/format.hpp>
#include <boost/foreach.hpp>

#include <Utils.hpp>
#include "USBRadio.hpp"
#include "USB_Device.hpp"
#include "cc1101.h"
#include "radio_config.h"

using namespace std;
using namespace boost;
using namespace Packet;

USBRadio::USBRadio()
{
	_sequence = 0;
	_device = 0;
	_printedError = false;
}

USBRadio::~USBRadio()
{
	if (_device)
	{
		delete _device;
	}
}

bool USBRadio::open()
{
	vector<USB_Device *> devs;
	USB_Device::find_all(devs, 0x3141, 0x0004);
	
	if (devs.empty())
	{
		if (!_printedError)
		{
			fprintf(stderr, "USBRadio: No devices found\n");
			_printedError = true;
		}
		return false;
	}
	
	_device = 0;
	BOOST_FOREACH(USB_Device *dev, devs)
	{
		if (dev->open())
		{
			_device = dev;
			break;
		}
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
	
	if (!_device->set_default())
	{
		if (!_printedError)
		{
			fprintf(stderr, "USBRadio: Can't set default config\n");
			_printedError = true;
		}
		return false;
	}
	
	configure();
	_printedError = false;
	
	return true;
}

void USBRadio::command(uint8_t cmd)
{
	if (!_device->control(USB_Device::Control_In | USB_Device::Control_Vendor, 2, 0, cmd))
	{
		throw runtime_error("USBRadio::command control write failed");
	}
}

void USBRadio::write(uint8_t reg, uint8_t value)
{
	if (!_device->control(USB_Device::Control_In | USB_Device::Control_Vendor, 1, value, reg))
	{
		throw runtime_error("USBRadio::write control write failed");
	}
}

uint8_t USBRadio::read(uint8_t reg)
{
	uint8_t value = 0;
	if (!_device->control(USB_Device::Control_In | USB_Device::Control_Vendor, 3, 0, reg, &value, 1))
	{
		throw runtime_error("USBRadio::read control write failed");
	}
	
	return value;
}

void USBRadio::reverse_size(int size)
{
	if (!_device->control(USB_Device::Control_In | USB_Device::Control_Vendor, 5, Reverse_Size, 0))
	{
		throw runtime_error("USBRadio::reverse_size control write failed");
	}
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

	reverse_size(Reverse_Size);

	auto_calibrate(true);
}

void USBRadio::auto_calibrate(bool enable)
{
	int flag = enable ? 1 : 0;
	assert(_device->control(USB_Device::Control_In | USB_Device::Control_Vendor, 4, flag, 0));
}

bool USBRadio::isOpen() const
{
	return _device;
}

void USBRadio::send(const Packet::RadioTx& packet)
{
	if (!_device)
	{
		if (!open())
		{
			return;
		}
	}
	
	uint8_t forward_packet[Forward_Size];
	
	int reverse_board_id = packet.reverse_board_id();
	
	// Build a forward packet
	forward_packet[0] = (_sequence << 4) | reverse_board_id;
	
	int offset = 1;
	int self_bots = 0;
	int robot_id;
	for (robot_id = 0; robot_id < 5 && robot_id < packet.robots_size(); ++robot_id)
	{
		//FIXME - Read from both channels and merge
		const RadioTx::Robot &robot = packet.robots(robot_id);
		int board_id = robot.board_id();
		
		int8_t m0, m1, m2, m3;
		uint8_t kick, roller;
		
		self_bots++;
		
		m1 = -robot.motors(0);
		m2 = -robot.motors(1);
		m3 = -robot.motors(2);
		m0 = -robot.motors(3);
		kick = robot.kick();
		
		if (robot.roller() > 0)
		{
			roller = robot.roller() * 2;
		} else {
			roller = 0;
		}
		
		forward_packet[offset++] = m0;
		forward_packet[offset++] = m1;
		forward_packet[offset++] = m2;
		forward_packet[offset++] = m3;
		forward_packet[offset++] = (roller & 0xf0) | (board_id & 0x0f);
		forward_packet[offset++] = kick;
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
	}
	
	// Send the forward packet
	if (!_device->bulk_write(1, forward_packet, sizeof(forward_packet)))
	{
		fprintf(stderr, "USBRadio: Bulk write failed\n");
		delete _device;
		_device = 0;
	}
	
	_sequence = (_sequence + 1) & 15;
}

bool USBRadio::receive(Packet::RadioRx* packet)
{
	if (!_device)
	{
		if (!open())
		{
			return false;
		}
	}
	
	uint8_t reverse_packet[Reverse_Size + 2];
	
	uint64_t rx_time = 0;
	// Read a forward packet if one is available
	//FIXME - Why is the timeout 1 instead of 0?
	if (!_device->bulk_read(2, reverse_packet, sizeof(reverse_packet), 1))
	{
		return false;
	}
	rx_time = Utils::timestamp();

	int board_id = reverse_packet[0] & 0x0f;
	
	packet->set_timestamp(rx_time);
	packet->set_board_id(board_id);
	packet->set_rssi((int8_t)reverse_packet[1] / 2.0);
	packet->set_battery(reverse_packet[3] * 3.3 / 256.0 * 5.0);
	packet->set_ball_sense(reverse_packet[5] & (1 << 5));
	packet->set_charged(reverse_packet[4] & 1);
	packet->set_motor_fault(reverse_packet[5] & 0x1f);
	
	for (int i = 0; i < 4; ++i)
	{
		int value = reverse_packet[6 + i] | ((reverse_packet[10] >> (i * 2)) & 3) << 8;
		packet->add_encoders(value);
	}
	
	return true;
}
