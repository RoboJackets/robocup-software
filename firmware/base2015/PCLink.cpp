#include <string>
#include "PCLink.hpp"

PCLink::PCLink(uint16_t vendorID, uint16_t productID, uint16_t release)
    : _usbLink(64, 64, vendorID, productID, release), _rxLed(NC), _txLed(NC) {
    _pc = nullptr;
}

bool PCLink::read() {
    if (_usbLink.readNB(&_in)) {
        _rxLed = 1;

        if (_pc != nullptr) {
            _pc->printf("%d\r\n", _in.data[0]);
        }

        _rxLed = 0;

        return true;
    }

    return false;
}
