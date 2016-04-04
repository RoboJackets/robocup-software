#include <mbed.h>
#include <cmsis_os.h>
#include <memory>

#include "PCLink.hpp"
#include "SharedSPI.hpp"
#include "CC1201Radio.hpp"
#include "pins.hpp"

using namespace std;

shared_ptr<RtosTimer> rx_led_ticker;
shared_ptr<RtosTimer> tx_led_ticker;

bool initRadio() {
    /// A shared spi bus used for the fpga and cc1201 radio
    shared_ptr<SharedSPI> sharedSPI =
        make_shared<SharedSPI>(RJ_SPI_MOSI, RJ_SPI_MISO, RJ_SPI_SCK);
    sharedSPI->format(8, 0);  // 8 bits per transfer

    // RX/TX leds
    auto rxTimeoutLED =
        make_shared<FlashingTimeoutLED>(DigitalOut(LED1, OpenDrain));
    auto txTimeoutLED =
        make_shared<FlashingTimeoutLED>(DigitalOut(LED2, OpenDrain));

    // Startup the CommModule interface
    CommModule::Instance = make_shared<CommModule>(rxTimeoutLED, txTimeoutLED);
    shared_ptr<CommModule> commModule = CommModule::Instance;

    // Create a new physical hardware communication link
    global_radio =
        new CC1201(sharedSPI, RJ_RADIO_nCS, RJ_RADIO_INT, preferredSettings,
                   sizeof(preferredSettings) / sizeof(registerSetting_t));

    return global_radio->isConnected();
}

int main() {
    PCLink pcLink;
    pcLink.setTxLed(LED1);
    pcLink.setRxLed(LED2);

    if (initRadio()) {
        LOG(INIT, "Radio interface ready on %3.2fMHz!", global_radio->freq());
    } else {
        LOG(FATAL, "No radio interface found!\r\n");
    }

    while (true) {
        wait(0.1);
        pcLink.read();
    }
}
