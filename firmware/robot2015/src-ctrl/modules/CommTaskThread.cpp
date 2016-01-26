#include <rtos.h>

#include <vector>

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

/*
 * Information about the radio protocol can be found at:
 * https://www.overleaf.com/2187548nsfdps
 */

void loopback_ack_pck(rtp::packet* p) {
    rtp::packet ack_pck = *p;
    ack_pck.ack(false);
    ack_pck.sfs(true);
    CommModule::send(ack_pck);
}

void legacy_rx_cb(rtp::packet* p) {
    if (p->payload.size()) {
        LOG(OK,
            "Legacy rx successful!\r\n"
            "    Received:\t'%s' (%u bytes)\r\n",
            p->payload.data(), p->payload.size());
    } else if (p->sfs()) {
        LOG(OK, "Legacy rx ACK successful!\r\n");
    } else {
        LOG(WARN, "Received empty packet on Legacy interface");
    }

    if (p->ack()) loopback_ack_pck(p);
}

/**
 * [rx_callback This is executed for a successfully received radio packet.]
 * @param p [none]
 */
void loopback_rx_cb(rtp::packet* p) {
    std::vector<uint16_t> duty_cycles;
    duty_cycles.assign(5, 100);
    for (int i = 0; i < duty_cycles.size(); ++i)
        duty_cycles.at(i) = 100 + 206 * i;

    if (p->payload.size()) {
        LOG(OK,
            "Loopback rx successful!\r\n"
            "    Received:\t'%s' (%u bytes)\r\n"
            "    ACK:\t%s\r\n",
            p->payload.data(), p->payload.size(), (p->ack() ? "SET" : "UNSET"));

        if (p->subclass() == 1) {
            uint16_t status_byte = FPGA::Instance()->set_duty_cycles(
                duty_cycles.data(), duty_cycles.size());

            // grab the bottom 4 bits
            status_byte &= 0x000F;
            // flip bits 1 & 2
            status_byte = (status_byte & 0x000C) |
                          ((status_byte >> 1) & 0x0001) |
                          ((status_byte << 1) & 0x0002);
            ;
            // bit 3 goes to the 6th position
            status_byte |= ((status_byte >> 2) << 5) & 0x0023;
            // bit 4 goes to the 8th position
            status_byte |= ((status_byte >> 3) << 7);
            // shift it all up a byte
            status_byte <<= 8;

            // All motors error LEDs
            MCP23017::write_mask(status_byte, 0xA300);

            // M1 error LED
            // MCP23017::write_mask(~(1 << (8 + 1)), 0xFF00);

            // M2 error LED
            // MCP23017::write_mask(~(1 << (8 + 0)), 0xFF00);

            // M3 error LED
            // MCP23017::write_mask(~(1 << (8 + 5)), 0xFF00);

            // M4 error LED
            // MCP23017::write_mask(~(1 << (8 + 7)), 0xFF00);
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
            "    Sent:\t'%s' (%u bytes)\r\n"
            "    ACK:\t%s\r\n",
            p->payload.data(), p->payload.size(), (p->ack() ? "SET" : "UNSET"));
    } else if (p->sfs()) {
        LOG(OK, "Loopback tx ACK successful!\r\n");
    } else {
        LOG(WARN, "Sent empty packet on loopback interface");
    }

    CommModule::receive(*p);
}

/**
 * [Task_CommCtrl description]
 * @param args [description]
 */
void Task_CommCtrl(void const* args) {
    const osThreadId* mainID = (const osThreadId*)args;

    // Store the thread's ID
    osThreadId threadID = Thread::gettid();
    ASSERT(threadID != nullptr);

    // Store our priority so we know what to reset it to if ever needed
    osPriority threadPriority = osThreadGetPriority(threadID);

    // Startup the CommModule interface
    CommModule::Init();

    // Setup some lights that will blink whenever we send/receive packets
    static const DigitalInOut tx_led(RJ_TX_LED, PIN_OUTPUT, OpenDrain, 1);
    static const DigitalInOut rx_led(RJ_RX_LED, PIN_OUTPUT, OpenDrain, 1);

    /* Uncomment the below DigitalOut lines and comment out the
     * ones above to use the mbed's on-board LEDs.
     */
    // static DigitalOut tx_led(LED3, 0);
    // static DigitalOut rx_led(LED2, 0);

    RtosTimer rx_led_ticker(commLightsTask_RX, osTimerPeriodic, (void*)&rx_led);
    RtosTimer tx_led_ticker(commLightsTask_TX, osTimerPeriodic, (void*)&tx_led);

    rx_led_ticker.start(80);
    tx_led_ticker.start(80);

    // Create a new physical hardware communication link
    CC1201 radio(RJ_SPI_BUS, RJ_RADIO_nCS, RJ_RADIO_INT, preferredSettings,
                 sizeof(preferredSettings) / sizeof(registerSetting_t));

    /*
     * Ports are always displayed in ascending (lowest -> highest) order
     * according
     * to its port number when using the console. Since most of everything is
     * static,
     * the CommModule methods can be used from almost anywhere.
     */
    if (radio.isConnected() == true) {
        LOG(INIT,
            "Radio interface ready on %3.2fMHz!\r\n    Thread ID:\t%u\r\n    "
            "Priority:\t%d",
            radio.freq(), threadID, threadPriority);

        // Open a socket for running tests across the link layer
        CommModule::RxHandler(&loopback_rx_cb, rtp::port::LINK);
        CommModule::TxHandler(&loopback_tx_cb, rtp::port::LINK);
        CommModule::openSocket(rtp::port::LINK);

        // The usual way of opening a port.
        CommModule::RxHandler(&loopback_rx_cb, rtp::port::DISCOVER);
        CommModule::TxHandler((CommLink*)&radio, &CommLink::sendPacket,
                              rtp::port::DISCOVER);
        CommModule::openSocket(rtp::port::DISCOVER);

        // This port won't open since there's no RX callback to invoke. The
        // packets are simply dropped.
        CommModule::RxHandler(&loopback_rx_cb, rtp::port::LOGGER);
        CommModule::TxHandler((CommLink*)&radio, &CommLink::sendPacket,
                              rtp::port::LOGGER);
        CommModule::openSocket(rtp::port::LOGGER);

        // Legacy port
        CommModule::TxHandler((CommLink*)&radio, &CommLink::sendPacket,
                              rtp::port::LEGACY);
        CommModule::RxHandler(&legacy_rx_cb, rtp::port::LEGACY);
        CommModule::openSocket(rtp::port::LEGACY);

        LOG(INIT, "%u sockets opened", CommModule::NumOpenSockets());

    } else {
        LOG(FATAL,
            "No radio interface found!\r\n"
            "    Terminating main radio thread.");

        // Always keep the link test port open regardless
        CommModule::RxHandler(&loopback_rx_cb, rtp::port::LINK);
        CommModule::TxHandler(&loopback_tx_cb, rtp::port::LINK);
        CommModule::openSocket(rtp::port::LINK);

        // Set the error flag - bit positions are pretty arbitruary as of now
        comm_err |= 1 << 1;

        // Set the error code's valid bit
        comm_err |= 1 << 0;

        while (CommModule::isReady() == false) {
            Thread::wait(50);
        }

        // Radio error LED
        MCP23017::write_mask(~(1 << (8 + 2)), 1 << (8 + 2));

        // signal back to main and wait until we're signaled to continue
        osSignalSet((osThreadId)mainID, MAIN_TASK_CONTINUE);
        Thread::signal_wait(SUB_TASK_CONTINUE, osWaitForever);

        while (true) {
            Thread::wait(5000);
            Thread::yield();
        }

        osThreadTerminate(threadID);
        return;
    }

    // Wait until the threads with the CommModule class are all started up
    // and ready
    while (CommModule::isReady() == false) {
        Thread::wait(50);
    }

    MCP23017::write_mask(1 << (8 + 2), 1 << (8 + 2));

    // Set the error code's valid bit
    comm_err |= 1 << 0;

    // signal back to main and wait until we're signaled to continue
    osSignalSet((osThreadId)mainID, MAIN_TASK_CONTINUE);
    Thread::signal_wait(SUB_TASK_CONTINUE, osWaitForever);

    rtp::packet pck1("motor trigger");
    pck1.port(rtp::port::LINK);
    pck1.subclass(1);
    pck1.address(LOOPBACK_ADDR);
    pck1.ack(false);

    std::vector<uint8_t> data;
    for (size_t i = 0; i < 8; ++i) data.push_back(i);

    // the size of the payload as the first byte
    data.insert(data.begin(), data.size());

    rtp::packet pck2(data);
    pck2.port(rtp::port::DISCOVER);
    pck2.subclass(1);
    pck2.address(BASE_STATION_ADDR);
    pck2.ack(false);

    while (true) {
        for (size_t i = 0; i < 60; ++i) {
            Thread::wait(2000);
            // CommModule::send(pck2);
            // CommModule::send(pck1);
            // Thread::wait(3);
            // CommModule::send(pck1);
            // Thread::wait(3);
            // CommModule::send(pck1);
            // Thread::wait(3);
            // CommModule::send(pck1);
        }
        Thread::wait(1000);
    }

    osThreadTerminate(threadID);
}
