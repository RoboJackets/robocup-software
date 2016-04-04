#include <mbed.h>
#include <cmsis_os.h>
#include "PCLink.hpp"

Serial pc(USBTX, USBRX);

int main() {
    PCLink pcLink;
    pcLink.setSerialDebugging(&pc);
    pcLink.setTxLed(LED1);
    pcLink.setRxLed(LED2);

    while (true) {
        wait(0.1);
        pcLink.read();
    }
}
