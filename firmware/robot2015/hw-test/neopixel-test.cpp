#include <cmath>

#include <mbed.h>

#include "robot-devices.hpp"
#include "neostrip.hpp"

DigitalOut ledOne(LED1, 1);

int main() {
    NeoStrip rgbLED(RJ_NEOPIXEL, 2);
    rgbLED.clear();
    rgbLED.brightness(0.1);

    int rb = 0;
    while (1) {
        ledOne = !ledOne;
        for (; rb < 0x100; ++rb) {
            int opp = 0xFF - rb;
            rgbLED.setPixel(0, rb, 0, opp);
            rgbLED.setPixel(1, opp, 0, rb);
            rgbLED.write();
            wait(0.005);
        }
        for (; rb >= 0; --rb) {
            int opp = 0xFF - rb;
            rgbLED.setPixel(0, rb, 0, opp);
            rgbLED.setPixel(1, opp, 0, rb);
            rgbLED.write();
            wait(0.005);
        }
    }
}
