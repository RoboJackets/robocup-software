#include <stdio.h>
#include <stdexcept>
#include <iostream>
#include <fstream>

#include "XBEERadio.hpp"

static const int Control_Timeout = 1000;

XBEERadio::XBEERadio() {

}

XBEERadio::XBEERadio(int id) {

}

XBEERadio::~XBEERadio() {

}


bool XBEERadio::isOpen() const {
    //TODO: WFU
    return true;
}

void XBEERadio::send(Packet::RadioTx& packet) {

}

void XBEERadio::receive() {

}

void XBEERadio::channel(int n) {

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


void XBEERadio::testonly() {

}




