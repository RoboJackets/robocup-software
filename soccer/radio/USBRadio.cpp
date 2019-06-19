#include <stdio.h>
#include <stdexcept>

#include <QMutexLocker>

#include <Utils.hpp>
#include "Geometry2d/Util.hpp"
#include "USBRadio.hpp"

// Include this file for base station usb vendor/product ids
#include "rc-fshare/usb-interface.hpp"
// included for kicer status enum
#include "status.h"

#include "PacketConvert.hpp"

using namespace std;
using namespace Packet;

// Timeout for control transfers, in milliseconds
static const int Control_Timeout = 10;

// Buffer to leave at the end of the decawave control packet to prevent our data
// from being corrupted
// https://github.com/thotro/arduino-dw1000/blob/511930c301f49b39b7b197acf52741d96155187a/src/DW1000.h#L197-L201
static const int Decawave_End_Buffer = 2;

USBRadio::USBRadio() : _mutex(QMutex::Recursive) {
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
        if (err == 0 && desc.idVendor == RJ_BASE_VENDOR_ID &&
            desc.idProduct == RJ_BASE_PRODUCT_ID) {
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
            _rxBuffers[i],     // data buffer
            rtp::ReverseSize,  // length of data buffer
            rxCompleted,       // callback function to be invoked on transfer
                               // completion
            this,              // user data to pass to callback function
            0);                // timeout for the transfer in milliseconds
        libusb_submit_transfer(_rxTransfers[i]);
    }

    _printedError = false;

    return true;
}

void USBRadio::rxCompleted(libusb_transfer* transfer) {
    USBRadio* radio = (USBRadio*)transfer->user_data;

    if (transfer->status == LIBUSB_TRANSFER_COMPLETED &&
        transfer->actual_length == rtp::ReverseSize) {
        // Parse the packet and add to the list of RadioRx's
        radio->handleRxData(transfer->buffer);
    }

    // Restart the transfer
    libusb_submit_transfer(transfer);
}

void USBRadio::command(uint8_t cmd) {
    if (libusb_control_transfer(_device,
                                LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR,
                                BaseControlCommand::RadioStrobe, 0, cmd,
                                nullptr, 0, Control_Timeout)) {
        throw runtime_error("USBRadio::command control write failed");
    }
}

void USBRadio::write(uint8_t reg, uint8_t value) {
    if (libusb_control_transfer(_device,
                                LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR,
                                BaseControlCommand::RadioWriteRegister, value,
                                reg, nullptr, 0, Control_Timeout)) {
        throw runtime_error("USBRadio::write control write failed");
    }
}

uint8_t USBRadio::read(uint8_t reg) {
    uint8_t value = 0;
    if (libusb_control_transfer(_device,
                                LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR,
                                BaseControlCommand::RadioReadRegister, 0, reg,
                                &value, 1, Control_Timeout)) {
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

    uint8_t forward_packet[rtp::ForwardSize];

    // ensure Forward_Size is correct
    static_assert(sizeof(rtp::Header) + 6 * sizeof(rtp::RobotTxMessage) ==
                      rtp::ForwardSize,
                  "Forward packet contents exceeds buffer size");

    fill_header(reinterpret_cast<rtp::Header*>(forward_packet));
    rtp::RobotTxMessage* body = reinterpret_cast<rtp::RobotTxMessage*>(
            forward_packet + sizeof(rtp::Header));

    //convert_tx_proto_to_rtp(packet, body, Robots_Per_Team);

    // Send the forward packet
    int sent = 0;
    // Leave a buffer at the end of every packet, so we don't get corruption
    // https://github.com/thotro/arduino-dw1000/blob/511930c301f49b39b7b197acf52741d96155187a/src/DW1000.h#L197-L201
    int transferRetCode = libusb_bulk_transfer(
        _device, LIBUSB_ENDPOINT_OUT | 2, forward_packet,
        sizeof(forward_packet) + Decawave_End_Buffer, &sent, Control_Timeout);
    if (transferRetCode != LIBUSB_SUCCESS ||
        sent != sizeof(forward_packet) + Decawave_End_Buffer) {
        fprintf(stderr, "USBRadio: Bulk write failed. sent = %d, size = %lu\n",
                sent, (unsigned long int)sizeof(forward_packet));
        if (transferRetCode != LIBUSB_SUCCESS)
            fprintf(stderr, "  Error: '%s'\n",
                    libusb_error_name(transferRetCode));

        int ret = libusb_clear_halt(_device, LIBUSB_ENDPOINT_OUT | 2);
        if (ret != 0) {
            printf("tried to clear halt, error = %s\n. closing device\n",
                   libusb_error_name(ret));
            libusb_close(_device);
            _device = nullptr;
        }
    }
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

// Note: this method assumes that sizeof(buf) == rtp::ReverseSize
void USBRadio::handleRxData(uint8_t* buf) {
    rtp::RobotStatusMessage* msg =
        reinterpret_cast<rtp::RobotStatusMessage*>(buf + sizeof(rtp::Header));

    RadioRx packet = convert_rx_rtp_to_proto(*msg);
    std::lock_guard<std::mutex> lock(_reverse_packets_mutex);
    _reversePackets.push_back(packet);
}

void USBRadio::channel(int n) {
    QMutexLocker lock(&_mutex);

    if (_device) {
        if (libusb_control_transfer(
                _device, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR,
                BaseControlCommand::RadioSetChannel, n, 0, nullptr, 0,
                Control_Timeout)) {
            throw runtime_error("USBRadio::channel control write failed");
        }
    }

    Radio::channel(n);
}
