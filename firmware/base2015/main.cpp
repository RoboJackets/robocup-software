//
// Created by matt on 6/22/15.
//
#include <mbed.h>
#include <cmsis_os.h>
#include "PCLink.hpp"

Serial pc(USBTX, USBRX);

// void hidRx_t(void const *args);

int main() {
    DigitalOut ledOne(LED1);
    DigitalOut ledTwo(LED2);
    PCLink defLink;
    // osThreadDef(hidRx_t, osPriorityNormal, DEFAULT_STACK_SIZE);
    // osThreadCreate(osThread(hidRx_t), NULL);

    defLink.setSerialDebugging(&pc);
    defLink.setLed(&ledTwo);
    while (true) {
        wait(0.1);
        defLink.read();
        ledOne = !ledOne;
    }
}

// void hidRx_t(void const *args) {
//
//}