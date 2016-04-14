#include <stdio.h>
#include <stdexcept>

#include <QMutexLocker>

#include <Utils.hpp>
#include "USBRadio.hpp"
#include "../firmware/common2015/drivers/cc1201/ti/defines.hpp"

// Include this file for base station usb vendor/product ids
#include "../firmware/base2015/usb-interface.hpp"

using namespace std;
using namespace Packet;

// Timeout for control transfers, in milliseconds
static const int Control_Timeout = 1000;

USBRadio::USBRadio() : _mutex(QMutex::Recursive) {
    _sequence = 0;
    _printedError = false;
    _device = nullptr;
    _usb_context = nullptr;
    libusb_init(&_usb_context);

    for (int i = 0; i < NumRXTransfers; ++i) {
        _rxTransfers[i] = libusb_alloc_transfer(0);
    }
}

USBRadio::~USBRadio() {
    if (_device) {
        libusb_close(_device);
    }

    for (int i = 0; i < NumRXTransfers; ++i) {
        libusb_free_transfer(_rxTransfers[i]);
    }

    libusb_exit(_usb_context);
}

bool USBRadio::open() {
    libusb_device** devices = nullptr;
    ssize_t numDevices = libusb_get_device_list(_usb_context, &devices);

    if (numDevices < 0) {
        fprintf(stderr, "libusb_get_device_list failed\n");
        return false;
    }

    int numRadios = 0;
    for (int i = 0; i < numDevices; ++i) {
        struct libusb_device_descriptor desc;
        int err = libusb_get_device_descriptor(devices[i], &desc);
        if (err == 0 && desc.idVendor == RJ_BASE2015_VENDOR_ID &&
            desc.idProduct == RJ_BASE2015_PRODUCT_ID) {
            ++numRadios;
            int err = libusb_open(devices[i], &_device);
            if (err == 0) {
                break;
            }
        }
    }

    libusb_free_device_list(devices, 1);

    if (!numRadios) {
        if (!_printedError) {
            fprintf(stderr, "USBRadio: No radio is connected\n");
            _printedError = true;
        }
        return false;
    }

    if (!_device) {
        if (!_printedError) {
            fprintf(stderr, "USBRadio: All radios are in use\n");
            _printedError = true;
        }
        return false;
    }

    if (libusb_set_configuration(_device, 1)) {
        if (!_printedError) {
            fprintf(stderr, "USBRadio: Can't set configuration\n");
            _printedError = true;
        }
        return false;
    }

    if (libusb_claim_interface(_device, 0)) {
        if (!_printedError) {
            fprintf(stderr, "USBRadio: Can't claim interface\n");
            _printedError = true;
        }
        return false;
    }

    channel(_channel);

    // Start the receive transfers
    for (int i = 0; i < NumRXTransfers; ++i) {
        // Populate the required libusb_transfer fields for a bulk transfer.
        libusb_fill_bulk_transfer(
            _rxTransfers[i],  // the transfer to populate
            _device,  // handle of the device that will handle the transfer
            LIBUSB_ENDPOINT_IN |
                2,  // address of the endpoint where this transfer will be sent
            _rxBuffers[i],          // data buffer
            rtp::Reverse_Size + 2,  // length of data buffer
            rxCompleted,  // callback function to be invoked on transfer
                          // completion
            this,         // user data to pass to callback function
            0);           // timeout for the transfer in milliseconds
        libusb_submit_transfer(_rxTransfers[i]);
    }

    _printedError = false;

    return true;
}

void USBRadio::rxCompleted(libusb_transfer* transfer) {
    USBRadio* radio = (USBRadio*)transfer->user_data;

    if (transfer->status == LIBUSB_TRANSFER_COMPLETED &&
        transfer->actual_length == rtp::Reverse_Size + 2) {
        // Parse the packet and add to the list of RadioRx's
        radio->handleRxData(transfer->buffer);
    } else {
        cerr << "bad rx from usbradio" << endl;  // TODO: remove
    }

    // Restart the transfer
    libusb_submit_transfer(transfer);
}

void USBRadio::command(uint8_t cmd) {
    if (libusb_control_transfer(_device,
                                LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR,
                                Base2015ControlCommand::RadioStrobe, 0, cmd,
                                nullptr, 0, Control_Timeout)) {
        throw runtime_error("USBRadio::command control write failed");
    }
}

void USBRadio::write(uint8_t reg, uint8_t value) {
    if (libusb_control_transfer(_device,
                                LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR,
                                Base2015ControlCommand::RadioWriteRegister,
                                value, reg, nullptr, 0, Control_Timeout)) {
        throw runtime_error("USBRadio::write control write failed");
    }
}

uint8_t USBRadio::read(uint8_t reg) {
    uint8_t value = 0;
    if (libusb_control_transfer(_device,
                                LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR,
                                Base2015ControlCommand::RadioReadRegister, 0,
                                reg, &value, 1, Control_Timeout)) {
        throw runtime_error("USBRadio::read control write failed");
    }

    return value;
}

bool USBRadio::isOpen() const { return _device; }

void USBRadio::send(Packet::RadioTx& packet) {
    QMutexLocker lock(&_mutex);
    if (!_device) {
        if (!open()) {
            return;
        }
    }

    uint8_t forward_packet[rtp::Forward_Size];

    // ensure Forward_Size is correct
    static_assert(sizeof(rtp::header_data) + 6 * sizeof(rtp::ControlMessage) ==
                      rtp::Forward_Size,
                  "Forward packet contents exceeds buffer size");

    // Unit conversions
    static const float Seconds_Per_Cycle = 0.005f;
    static const float Meters_Per_Tick = 0.026f * 2 * M_PI / 6480.0f;
    static const float Radians_Per_Tick = 0.026f * M_PI / (0.0812f * 3240.0f);

    rtp::header_data* header = (rtp::header_data*)forward_packet;
    header->port = rtp::Port::CONTROL;
    header->address = rtp::BROADCAST_ADDRESS;
    header->type = rtp::header_data::Type::Control;

    // Build a forward packet
    for (int slot = 0; slot < 6; ++slot) {
        // Calculate the offset into the @forward_packet for this robot's
        // control message and cast it to a ControlMessage pointer for easy
        // access
        size_t offset =
            sizeof(rtp::header_data) + slot * sizeof(rtp::ControlMessage);
        rtp::ControlMessage* msg =
            (rtp::ControlMessage*)(forward_packet + offset);

        if (slot < packet.robots_size()) {
            const Packet::Control& robot = packet.robots(slot).control();

            // TODO: clarify units/resolution
            float bodyVelX = robot.xvelocity() * Seconds_Per_Cycle /
                             Meters_Per_Tick / sqrtf(2);
            float bodyVelY = robot.yvelocity() * Seconds_Per_Cycle /
                             Meters_Per_Tick / sqrtf(2);
            float bodyVelW =
                robot.avelocity() * Seconds_Per_Cycle / Radians_Per_Tick;

            msg->uid = 2;  // packet.robots(slot).uid(); // TODO(justin): fix

            msg->bodyX = clamp((int)roundf(bodyVelX), -511, 511);
            msg->bodyY = clamp((int)roundf(bodyVelY), -511, 511);
            msg->bodyW = clamp((int)roundf(bodyVelW), -511, 511);

            msg->dribbler =
                max(0, min(255, static_cast<uint16_t>(robot.dvelocity()) * 2));

            msg->kickStrength = robot.kcstrength();

            msg->shootMode = robot.shootmode();
            msg->triggerMode = robot.triggermode();
            msg->song = robot.song();
        } else {
            // empty slot
            msg->uid = 2;
        }
    }

    // Send the forward packet
    int sent = 0;
    int transferRetCode =
        libusb_bulk_transfer(_device, LIBUSB_ENDPOINT_OUT | 2, forward_packet,
                             sizeof(forward_packet), &sent, Control_Timeout);
    if (transferRetCode != LIBUSB_SUCCESS || sent != sizeof(forward_packet)) {
        fprintf(stderr, "USBRadio: Bulk write failed\n");
        if (transferRetCode != LIBUSB_SUCCESS)
            fprintf(stderr, "  Error: '%s'\n",
                    libusb_error_name(transferRetCode));

        libusb_close(_device);
        _device = nullptr;
    }

    _sequence = (_sequence + 1) & 7;
}

void USBRadio::receive() {
    QMutexLocker lock(&_mutex);

    if (!_device) {
        if (!open()) {
            return;
        }
    }

    // Handle USB events.  This will call callbacks.
    struct timeval tv = {0, 0};
    libusb_handle_events_timeout(_usb_context, &tv);
}

void USBRadio::handleRxData(uint8_t* buf) {
    RJ::Time rx_time = RJ::timestamp();

    // TODO(justin): should the packet be populated before putting into the
    // queue?

    _reversePackets.push_back(RadioRx());
    RadioRx& packet = _reversePackets.back();

    // TODO(justin): check size of buf?
    rtp::RobotStatusMessage* msg = (rtp::RobotStatusMessage*)buf;

    // TODO(justin): add back missing fields

    packet.set_timestamp(rx_time);
    packet.set_robot_id(msg->uid);
    // packet.set_rssi((int8_t)buf[1] / 2.0 - 74);
    packet.set_battery(msg->battVoltage);
// packet.set_kicker_status(buf[3]);

// // Drive motor status
// for (int i = 0; i < 4; ++i) {
//     packet.add_motor_status(MotorStatus((buf[4] >> (i * 2)) & 3));
// }

// Dribbler status
// packet.add_motor_status(MotorStatus(buf[5] & 3));

// Hardware version
// if (buf[5] & (1 << 4)) {
//     packet.set_hardware_version(RJ2008);
// } else {
//     packet.set_hardware_version(RJ2011);
// }

// packet.set_ball_sense_status(BallSenseStatus((buf[5] >> 2) & 3));
// packet.set_kicker_voltage(buf[6]);

#if 0
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
#endif

#if 0
	if (buf[5] & (1 << 5))
	{
		// Quaternion
		int16_t q0 = buf[7] | (buf[8] << 8);
		int16_t q1 = buf[9] | (buf[10] << 8);
		int16_t q2 = buf[11] | (buf[12] << 8);
		int16_t q3 = buf[13] | (buf[14] << 8);
		packet.mutable_quaternion()->set_q0(q0 / 16384.0);
		packet.mutable_quaternion()->set_q1(q1 / 16384.0);
		packet.mutable_quaternion()->set_q2(q2 / 16384.0);
		packet.mutable_quaternion()->set_q3(q3 / 16384.0);
	}
#endif
}

void USBRadio::channel(int n) {
    QMutexLocker lock(&_mutex);

    if (_device) {
        // TODO(justin): fix
        // write(CHANNR, n);
        // throw std::runtime_error("Channel-setting not implemented for
        // cc1201");

        command(CC1201_STROBE_SIDLE);
        command(CC1201_STROBE_SRX);
    }

    Radio::channel(n);
}
