#include <rtos.h>

#include <vector>
#include <memory>

#include <CommModule.hpp>
#include <CommPort.hpp>
#include <CC1201Radio.hpp>
#include <helper-funcs.hpp>
#include <logger.hpp>
#include <assert.hpp>

#include "robot-devices.hpp"
#include "task-signals.hpp"
#include "io-expander.hpp"
#include "fpga.hpp"
#include "TimeoutLED.hpp"

using namespace std;

/*
 * Information about the radio protocol can be found at:
 * https://www.overleaf.com/2187548nsfdps
 */
void loopback_ack_pck(rtp::packet* p) {
    rtp::packet ack_pck = *p;
    CommModule::Instance->send(ack_pck);
}

void legacy_rx_cb(rtp::packet* p) {
    if (p->payload.size()) {
        LOG(OK,
            "Legacy rx successful!\r\n"
            "    Received: %u bytes\r\n",
            p->payload.size());
    } else {
        LOG(WARN, "Received empty packet on Legacy interface");
    }
}

void loopback_rx_cb(rtp::packet* p) {
    vector<uint16_t> duty_cycles;
    duty_cycles.assign(5, 100);
    for (size_t i = 0; i < duty_cycles.size(); ++i)
        duty_cycles.at(i) = 100 + 206 * i;

    if (p->payload.size()) {
        LOG(OK,
            "Loopback rx successful!\r\n"
            "    Received: %u bytes",
            p->payload.size());
    } else {
        LOG(WARN, "Received empty packet on loopback interface");
    }
}

void loopback_tx_cb(rtp::packet* p) {
    if (p->payload.size()) {
        LOG(OK,
            "Loopback tx successful!\r\n"
            "    Sent: %u bytes\r\n",
            p->payload.size());
    } else {
        LOG(WARN, "Sent empty packet on loopback interface");
    }

    CommModule::Instance->receive(*p);
}

// Setup some lights that will blink whenever we send/receive packets
const DigitalInOut tx_led(RJ_TX_LED, PIN_OUTPUT, OpenDrain, 1);
const DigitalInOut rx_led(RJ_RX_LED, PIN_OUTPUT, OpenDrain, 1);

shared_ptr<RtosTimer> rx_led_ticker;
shared_ptr<RtosTimer> tx_led_ticker;

void InitializeCommModule() {
    // leds that flash if tx/rx have happened recently
    auto rxTimeoutLED =
        make_shared<FlashingTimeoutLED>(DigitalOut(RJ_RX_LED, OpenDrain));
    auto txTimeoutLED =
        make_shared<FlashingTimeoutLED>(DigitalOut(RJ_TX_LED, OpenDrain));

    // Startup the CommModule interface
    CommModule::Instance = make_shared<CommModule>(rxTimeoutLED, txTimeoutLED);
    shared_ptr<CommModule> commModule = CommModule::Instance;

    // TODO(justin): make this non-global
    // Create a new physical hardware communication link
    global_radio =
        new CC1201(RJ_SPI_BUS, RJ_RADIO_nCS, RJ_RADIO_INT, preferredSettings,
                   sizeof(preferredSettings) / sizeof(registerSetting_t));

    // Open a socket for running tests across the link layer
    // The LINK port handlers are always active, regardless of whether or not a
    // working radio is connected.
    commModule->setRxHandler(&loopback_rx_cb, rtp::port::LINK);
    commModule->setTxHandler(&loopback_tx_cb, rtp::port::LINK);

    /*
     * Ports are always displayed in ascending (lowest -> highest) order
     * according to its port number when using the console.
     */
    if (global_radio->isConnected() == true) {
        LOG(INIT, "Radio interface ready on %3.2fMHz!", global_radio->freq());

        // The usual way of opening a port.
        commModule->setRxHandler(&loopback_rx_cb, rtp::port::DISCOVER);
        commModule->setTxHandler((CommLink*)global_radio, &CommLink::sendPacket,
                                 rtp::port::DISCOVER);

        // This port won't open since there's no RX callback to invoke. The
        // packets are simply dropped.
        commModule->setRxHandler(&loopback_rx_cb, rtp::port::LOGGER);
        commModule->setTxHandler((CommLink*)global_radio, &CommLink::sendPacket,
                                 rtp::port::LOGGER);

        // Legacy port
        commModule->setTxHandler((CommLink*)global_radio, &CommLink::sendPacket,
                                 rtp::port::LEGACY);
        commModule->setRxHandler(&legacy_rx_cb, rtp::port::LEGACY);

        LOG(INIT, "%u sockets opened", commModule->numOpenSockets());

        // Wait until the threads with the commModule are all started up
        // and ready
        while (!commModule->isReady()) {
            Thread::wait(50);
        }
    } else {
        LOG(FATAL, "No radio interface found!\r\n");

        // Wait until the threads with the commModule are all started up
        // and ready
        while (!commModule->isReady()) {
            Thread::wait(50);
        }
    }
}
