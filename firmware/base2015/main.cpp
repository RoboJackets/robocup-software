#include <cmsis_os.h>
#include <mbed.h>
#include <memory>

#include "CC1201Radio.hpp"
#include "RJBaseUSBDevice.hpp"
#include "SharedSPI.hpp"
#include "logger.hpp"
#include "pins.hpp"
#include "usb-interface.hpp"
#include "watchdog.hpp"
#include "logger.hpp"

#define RJ_WATCHDOG_TIMER_VALUE 2  // seconds

using namespace std;

// setup USB interface with custom vendor/product ids
RJBaseUSBDevice usbLink(RJ_BASE2015_VENDOR_ID, RJ_BASE2015_PRODUCT_ID,
                        RJ_BASE2015_RELEASE);

bool initRadio() {
    // setup SPI bus
    shared_ptr<SharedSPI> sharedSPI =
        make_shared<SharedSPI>(RJ_SPI_MOSI, RJ_SPI_MISO, RJ_SPI_SCK);
    sharedSPI->format(8, 0);  // 8 bits per transfer

    // RX/TX leds
    auto rxTimeoutLED = make_shared<FlashingTimeoutLED>(LED1);
    auto txTimeoutLED = make_shared<FlashingTimeoutLED>(LED2);

    // Startup the CommModule interface
    CommModule::Instance = make_shared<CommModule>(rxTimeoutLED, txTimeoutLED);

    // Create a new physical hardware communication link
    global_radio =
        new CC1201(sharedSPI, RJ_RADIO_nCS, RJ_RADIO_INT, preferredSettings,
                   sizeof(preferredSettings) / sizeof(registerSetting_t));

    return global_radio->isConnected();
}

void radioRxHandler(rtp::packet pkt) {
    LOG(INF3, "radioRxHandler()");
    // write packet content (including header) out to EPBULK_IN
    vector<uint8_t> buf;
    pkt.pack(&buf);

    // drop the packet if it's the wrong size. Thsi will need to be changed if we have variable-sized reply packets
    if (buf.size() != rtp::Reverse_Size) {
        LOG(WARN, "Dropping packet, wrong size '%u', should be '%u'", buf.size(), rtp::Reverse_Size);
        return;
    }

    bool success = usbLink.writeNB(EPBULK_IN, buf.data(), buf.size(),
                                   MAX_PACKET_SIZE_EPBULK);

    // TODO(justin): add this message back in. For some reason, the usb system
    // reports failure *unless* I add a print statement inside
    // USBDevice.writeNB() after result = endpointWrite().  No idea why this is
    // the case
    //
    // if (!success) LOG(WARN, "Failed to transfer received %u byte packet over
    // usb", pkt.payload.size());
}

int main() {
    // set baud rate to higher value than the default for faster terminal
    Serial s(RJ_SERIAL_RXTX);
    s.baud(57600);

    // Set the default logging configurations
    isLogging = RJ_LOGGING_EN;
    rjLogLevel = INIT;

    printf("****************************************\r\n");
    LOG(INIT, "Base station starting...");

    if (initRadio()) {
        LOG(INIT, "Radio interface ready on %3.2fMHz!", global_radio->freq());

        // register handlers for any ports we might use
        for (rtp::Port port :
             {rtp::Port::CONTROL, rtp::Port::PING, rtp::Port::LEGACY}) {
            CommModule::Instance->setRxHandler(&radioRxHandler, port);
            CommModule::Instance->setTxHandler((CommLink*)global_radio,
                                               &CommLink::sendPacket, port);
        }
    } else {
        LOG(FATAL, "No radio interface found!");
    }

    global_radio->setAddress(rtp::BASE_STATION_ADDRESS);

    DigitalOut radioStatusLed(LED4, global_radio->isConnected());

    // set callbacks for usb control transfers
    usbLink.writeRegisterCallback =
        [](uint8_t reg, uint8_t val) { global_radio->writeReg(reg, val); };
    usbLink.readRegisterCallback =
        [](uint8_t reg) { return global_radio->readReg(reg); };
    usbLink.strobeCallback =
        [](uint8_t strobe) { global_radio->strobe(strobe); };
    usbLink.setRadioChannelCallback = [](uint8_t chanNumber) {
        global_radio->setChannel(chanNumber);
        LOG(INIT, "Set radio channel to %u", chanNumber);
    };

    LOG(INIT, "Initializing USB interface...");
    usbLink.connect();  // note: this blocks until the link is connected
    LOG(INIT, "Initialized USB interface!");

    // Set the watdog timer's initial config
    Watchdog::Set(RJ_WATCHDOG_TIMER_VALUE);

    LOG(INIT, "Listening for commands over USB");

    // buffer to read data from usb bulk transfers into
    uint8_t buf[MAX_PACKET_SIZE_EPBULK];
    uint32_t bufSize;

    while (true) {
        // make sure we can always reach back to main by renewing the watchdog
        // timer periodically
        Watchdog::Renew();

        // attempt to read data from EPBULK_OUT
        // if data is available, write it into @pkt and send it
        if (usbLink.readEP_NB(EPBULK_OUT, buf, &bufSize,
                              MAX_PACKET_SIZE_EPBULK)) {
            LOG(INF3, "Read %d bytes from BULK IN", bufSize);

            // construct packet from buffer received over USB
            rtp::packet pkt;
            pkt.recv(buf, bufSize);

            // send to all robots
            pkt.header.address = rtp::ROBOT_ADDRESS;

            // transmit!
            CommModule::Instance->send(std::move(pkt));
        }
    }
}
