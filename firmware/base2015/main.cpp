#include <mbed.h>

int main() {
    DigitalOut ledOne(LED1);
    while (true) {
        wait(0.5);
        ledOne = !ledOne;
    }
}
