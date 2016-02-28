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
#include "task-globals.hpp"
#include "io-expander.hpp"
#include "fpga.hpp"

using namespace std;

/*
 * Information about the radio protocol can be found at:
 * https://www.overleaf.com/2187548nsfdps
 */

void loopback_ack_pck(rtp::packet* p) {
    rtp::packet ack_pck = *p;
    ack_pck.ack(false);
    ack_pck.sfs(true);
    CommModule::Instance()->send(ack_pck);
}

void legacy_rx_cb(rtp::packet* p) {
    if (p->payload.size()) {
        LOG(OK,
            "Legacy rx successful!\r\n"
            "    Received: %u bytes\r\n",
            p->payload.size());
    } else if (p->sfs()) {
        LOG(OK, "Legacy rx ACK successful!\r\n");
    } else {
        LOG(WARN, "Received empty packet on Legacy interface");
    }

    if (p->ack()) loopback_ack_pck(p);
}

void loopback_rx_cb(rtp::packet* p) {
    vector<uint16_t> duty_cycles;
    duty_cycles.assign(5, 100);
    for (size_t i = 0; i < duty_cycles.size(); ++i)
        duty_cycles.at(i) = 100 + 206 * i;

    if (p->payload.size()) {
        LOG(OK,
            "Loopback rx successful!\r\n"
            "    Received: %u bytes\r\n"
            "    ACK:\t%s\r\n",
            p->payload.size(), (p->ack() ? "SET" : "UNSET"));

        if (p->subclass() == 1) {
            uint16_t status_byte = FPGA::Instance()->set_duty_cycles(
                duty_cycles.data(), duty_cycles.size());

            // grab the bottom 4 bits
            status_byte &= 0x000F;
            // flip bits 1 & 2
            status_byte = (status_byte & 0x000C) |
                          ((status_byte >> 1) & 0x0001) |
                          ((status_byte << 1) & 0x0002);
            
            // bit 3 goes to the 6th position
            status_byte |= ((status_byte >> 2) << 5) & 0x0023;
            // bit 4 goes to the 8th position
            status_byte |= ((status_byte >> 3) << 7);
            // shift it all up a byte
            status_byte <<= 8;

            // All motors error LEDs
            MCP23017::Instance()->writeMask(status_byte, 0xA300);

            // M1 error LED
            // MCP23017::Instance()->writeMask(~(1 << (8 + 1)), 0xFF00);

            // M2 error LED
            // MCP23017::Instance()->writeMask(~(1 << (8 + 0)), 0xFF00);

            // M3 error LED
            // MCP23017::Instance()->writeMask(~(1 << (8 + 5)), 0xFF00);

            // M4 error LED
            // MCP23017::Instance()->writeMask(~(1 << (8 + 7)), 0xFF00);
        }
    } else if (p->sfs()) {
        LOG(OK, "Loopback rx ACK successful!\r\n");
    } else {
        LOG(WARN, "Received empty packet on loopback interface");
    }

    if (p->ack()) loopback_ack_pck(p);
}

void loopback_tx_cb(rtp::packet* p) {
    if (p->payload.size()) {
        LOG(OK,
            "Loopback tx successful!\r\n"
            "    Sent: %u bytes\r\n"
            "    ACK:\t%s\r\n",
            p->payload.size(), (p->ack() ? "SET" : "UNSET"));
    } else if (p->sfs()) {
        LOG(OK, "Loopback tx ACK successful!\r\n");
    } else {
        LOG(WARN, "Sent empty packet on loopback interface");
    }

    CommModule::Instance()->receive(*p);
}


/* Uncomment the below DigitalOut lines and comment out the
 * ones above to use the mbed's on-board LEDs.
 */
// static DigitalOut tx_led(LED3, 0);
// static DigitalOut rx_led(LED2, 0);
// Setup some lights that will blink whenever we send/receive packets
const DigitalInOut tx_led(RJ_TX_LED, PIN_OUTPUT, OpenDrain, 1);
const DigitalInOut rx_led(RJ_RX_LED, PIN_OUTPUT, OpenDrain, 1);

shared_ptr<RtosTimer> rx_led_ticker;
shared_ptr<RtosTimer> tx_led_ticker;

void InitializeCommModule() {
    // Startup the CommModule interface
    shared_ptr<CommModule> commModule = CommModule::Instance();

    // initialize and start LED ticker timers
    rx_led_ticker = make_shared<RtosTimer>(commLightsTask_RX, osTimerPeriodic, (void*)&rx_led);
    tx_led_ticker = make_shared<RtosTimer>(commLightsTask_TX, osTimerPeriodic, (void*)&tx_led);
    rx_led_ticker->start(80);
    tx_led_ticker->start(80);

    // TODO(justin): remove this
    // Create a new physical hardware communication link
    global_radio = new CC1201(RJ_SPI_BUS, RJ_RADIO_nCS, RJ_RADIO_INT, preferredSettings,
                 sizeof(preferredSettings) / sizeof(registerSetting_t));

    // Open a socket for running tests across the link layer
    // The LINK port handlers are always active, regardless of whether or not a
    // working radio is connected.
    commModule->setRxHandler(&loopback_rx_cb, rtp::port::LINK);
    commModule->setTxHandler(&loopback_tx_cb, rtp::port::LINK);
    commModule->openSocket(rtp::port::LINK);

    /*
     * Ports are always displayed in ascending (lowest -> highest) order
     * according
     * to its port number when using the console. Since most of everything is
     * static,
     * the CommModule methods can be used from almost anywhere.
     */
    if (global_radio->isConnected() == true) {
        LOG(INIT,
            "Radio interface ready on %3.2fMHz!\r\n",
            global_radio->freq());

        // The usual way of opening a port.
        commModule->setRxHandler(&loopback_rx_cb, rtp::port::DISCOVER);
        commModule->setTxHandler((CommLink*)global_radio, &CommLink::sendPacket,
                                 rtp::port::DISCOVER);
        commModule->openSocket(rtp::port::DISCOVER);

        // This port won't open since there's no RX callback to invoke. The
        // packets are simply dropped.
        commModule->setRxHandler(&loopback_rx_cb, rtp::port::LOGGER);
        commModule->setTxHandler((CommLink*)global_radio, &CommLink::sendPacket,
                                 rtp::port::LOGGER);
        commModule->openSocket(rtp::port::LOGGER);

        // Legacy port
        commModule->setTxHandler((CommLink*)global_radio, &CommLink::sendPacket,
                                 rtp::port::LEGACY);
        commModule->setRxHandler(&legacy_rx_cb, rtp::port::LEGACY);
        commModule->openSocket(rtp::port::LEGACY);

        LOG(INIT, "%u sockets opened", commModule->numOpenSockets());

        // Wait until the threads with the commModule->lass are all started up
        // and ready
        while (!commModule->isReady()) {
            Thread::wait(50);
        }

        MCP23017::Instance()->writeMask(1 << (8 + 2), 1 << (8 + 2));

        // Set the error code's valid bit
        comm_err |= 1 << 0;

    } else {
        LOG(FATAL, "No radio interface found!\r\n");

        // Set the error flag - bit positions are pretty arbitruary as of now
        comm_err |= 1 << 1;

        // Set the error code's valid bit
        comm_err |= 1 << 0;

        while (!commModule->isReady()) {
            Thread::wait(50);
        }

        // Radio error LED
        MCP23017::Instance()->writeMask(~(1 << (8 + 2)), 1 << (8 + 2));
    }
}
