#include <stdio.h>
#include <stdexcept>

#include "XBEERadio.hpp"

static const int Control_Timeout = 1000;

XBEERadio::XBEERadio() {

}

XBEERadio::~XBEERadio() {

}


bool XBEERadio::open() {
    //TODO: WFU
    return true;
}


void XBEERadio::command(uint8_t cmd) {

}

void XBEERadio::write(uint8_t reg, uint8_t value) {

}

uint8_t XBEERadio::read(uint8_t reg) {
    //TODO: wfu
    return 0;
}


