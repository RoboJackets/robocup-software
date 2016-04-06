#include <mbed.h>
#include <cmsis_os.h>
#include <memory>

#include "SharedSPI.hpp"
#include "CC1201Radio.hpp"
#include "logger.hpp"
#include "pins.hpp"
#include "usb-interface.hpp"
#include "watchdog.hpp"
#include "RJBaseHID.hpp"

#define RJ_WATCHDOG_TIMER_VALUE 2  // seconds

using namespace std;

// USBHID interface.  The false at the end tells it not to connect initially
RJBaseHID usbLink(RJ_BASE2015_VENDOR_ID, RJ_BASE2015_PRODUCT_ID,
                  RJ_BASE2015_RELEASE);

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

void radioRxHandler(rtp::packet* pkt) {
    // TODO: copy data
    HID_REPORT data;
    // bool success = usbLink.sendNB(&data);

    // if (!success) {
    //     LOG(WARN, "Failed to transfer received packet over usb");
    // }
}

int main() {
    // set baud rate to higher value than the default for faster terminal
    Serial s(RJ_SERIAL_RXTX);
    s.baud(57600);

    // Set the default logging configurations
    isLogging = RJ_LOGGING_EN;
    rjLogLevel = INIT;

    LOG(INIT, "Base station starting...");

    LOG(INIT, "Initializing USBHID interface...");
    usbLink.connect();  // note: this blocks until the link is connected
    LOG(INIT, "Initialized USBHID interface!");

    if (initRadio()) {
        LOG(INIT, "Radio interface ready on %3.2fMHz!", global_radio->freq());

        CommModule::Instance->setRxHandler(&radioRxHandler, rtp::port::CONTROL);
        CommModule::Instance->setTxHandler(
            (CommLink*)global_radio, &CommLink::sendPacket, rtp::port::CONTROL);
    } else {
        LOG(FATAL, "No radio interface found!");
    }

    DigitalOut radioStatusLed(LED4, global_radio->isConnected());

    // Set the watdog timer's initial config
    Watchdog::Set(RJ_WATCHDOG_TIMER_VALUE);

    LOG(INIT, "Listening for commands over USB");

    while (true) {
        // make sure we can always reach back to main by renewing the watchdog
        // timer periodically
        Watchdog::Renew();

        // HID_REPORT data;
        // if (usbLink.readNB(&data)) {
        //     // rtp::packet pkt;  // TODO: fill data
        //     // CommModule::Instance->send(pkt);

        //     printf("Got data\r\n");
        //     for (size_t i = 1; i < data.length; i++) {
        //         printf("%c", (char)data.data[i]);
        //     }
        //     printf("\r\n");
        // }
    }
}
