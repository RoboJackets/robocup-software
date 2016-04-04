#include <string>
#include "PCLink.hpp"

PCLink::PCLink(uint16_t vendorID, uint16_t productID, uint16_t release)
    : _usbLink(64, 64, vendorID, productID, release) {
    _pc = nullptr;
}

void PCLink::setSerialDebugging(Serial* pc) { _pc = pc; }

void PCLink::read() {
    if (usbLink.readNB(&_in)) {
        _led = !_led;

        if (_pc != nullptr) {
            _pc->printf("%d\r\n", _in.data[0]);
        }
    }
}
