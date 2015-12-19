#include <rtos.h>

#include <CommModule.hpp>
#include <CommPort.hpp>
#include <CC1201Radio.hpp>
#include <CC1201Config.hpp>
#include <helper-funcs.hpp>
#include <logger.hpp>
#include <assert.hpp>

#include "robot-devices.hpp"
#include "task-signals.hpp"

/*
 * Information about the radio protocol can be found at:
 * https://www.overleaf.com/2187548nsfdps
 */

/*
* Example of sending a packet.

rtp::packet pck;                // Declare a new packet structure

pck.port = 8;                   // What port should the packet be routed to?
pck.subclass = 1;               // What subclass of the port is this for?
pck.ack = false;                // Do we need an acknowledgment or is this an
acknowledgment response?
pck.sfs = false;                // Is the size field Significant? (Almost always
'No')

// The lines above can be set with the 'RTP_HEADER' macro as an alternative.
// RTP_HEADER(port, subclass, ack, sfs)
//
// pck.header_link = RTP_HEADER(rtp::packet::port::DISCOVER, 1, false, false);

pck.address = 255;              // Who are we sending the packet to? (255 is
broadcast address)
pck.payload_size = 25;          // How many bytes are in the payload of the
packet?

memset(pck.payload, 0xFF, pck.payload_size);    // fill with 25 arbitrary bytes

CommModule.send(pck);           // Send it!

// Note: A packet with a requested ACK must be accounted for within the RX
callback function.
*/

void loopback_ack_pck(rtp::packet* p) {
    rtp::packet ack_pck;
    memcpy(&ack_pck, p, p->total_size);
    ack_pck.ack = false;
    CommModule::send(ack_pck);
}

/**
 * [rx_callback This is executed for a successfully received radio packet.]
 * @param p [none]
 */
void loopback_rx_cb(rtp::packet* p) {
    if (p->payload_size) {
        LOG(OK,
            "Loopback rx successful!\r\n"
            "    Received:\t'%s' (%u bytes)\r\n"
            "    ACK:\t%s\r\n",
            p->payload, p->payload_size, (p->ack ? "SET" : "UNSET"));
    } else {
        LOG(WARN, "Received empty packet on loopback interface");
    }
    if (p->ack) loopback_ack_pck(p);
}

void loopback_tx_cb(rtp::packet* p) {
    if (p->payload_size) {
        LOG(OK,
            "Loopback tx successful!\r\n"
            "    Sent:\t'%s' (%u bytes)\r\n"
            "    ACK:\t%s\r\n",
            p->payload, p->payload_size, (p->ack ? "SET" : "UNSET"));
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
    DigitalInOut tx_led(RJ_TX_LED, PIN_OUTPUT, OpenDrain, 1);
    DigitalInOut rx_led(RJ_RX_LED, PIN_OUTPUT, OpenDrain, 1);
    CommModule::rxLED(&rx_led);
    CommModule::txLED(&tx_led);

    // Create a new physical hardware communication link
    CC1201 radio(RJ_SPI_BUS, RJ_RADIO_nCS, RJ_RADIO_INT);

    /*
     * Ports are always displayed in ascending (lowest -> highest) order
     * according
     * to its port number when using the console. Since most of everything is
     * static,
     * the CommModule methods can be used from almost anywhere.
     */
    if (radio.isConnected() == true) {
        // Load the configuration onto the radio transceiver
        CC1201Config* radioConfig = new CC1201Config();
        radioConfig = CC1201Config::resetConfiguration(radioConfig);
        CC1201Config::loadConfiguration(radioConfig, &radio);
        CC1201Config::verifyConfiguration(radioConfig, &radio);

        LOG(INIT,
            "Radio interface ready on %3.2fMHz!\r\n    Thread ID:\t%u\r\n    "
            "Priority:\t%d",
            radio.freq(), threadID, threadPriority);

        // Open a socket for running tests across the link layer
        CommModule::RxHandler(&loopback_rx_cb, rtp::port::LINK);
        CommModule::TxHandler(&loopback_tx_cb, rtp::port::LINK);
        // returns true if port was successfully opened.
        CommModule::openSocket(rtp::port::LINK);

        // The usual way of opening a port.
        CommModule::RxHandler(&loopback_rx_cb, rtp::port::DISCOVER);
        CommModule::TxHandler((CommLink*)&radio, &CommLink::sendPacket,
                              rtp::port::DISCOVER);
        CommModule::openSocket(rtp::port::DISCOVER);

        // This port won't open since there's no RX callback to invoke. The
        // packets are simply dropped.
        CommModule::RxHandler(&loopback_rx_cb, rtp::port::LOG);
        CommModule::TxHandler((CommLink*)&radio, &CommLink::sendPacket,
                              rtp::port::LOG);
        CommModule::openSocket(rtp::port::LOG);

        // There's no TX callback for this port, but it will still open when
        // invoked since it knows where to send an RX packet.
        CommModule::TxHandler((CommLink*)&radio, &CommLink::sendPacket,
                              rtp::port::SETPOINT);
        CommModule::RxHandler(&loopback_rx_cb, rtp::port::SETPOINT);
        CommModule::openSocket(rtp::port::SETPOINT);

        LOG(INIT, "%u sockets opened", CommModule::NumOpenSockets());

    } else {
        LOG(FATAL,
            "No radio interface found!\r\n"
            "\tTerminating main radio thread.");

        // TODO: Turn on radio error LED here

        // Always keep the link test port open regardless
        CommModule::RxHandler(&loopback_rx_cb, rtp::port::LINK);
        CommModule::TxHandler(&loopback_tx_cb, rtp::port::LINK);
        CommModule::openSocket(rtp::port::LINK);

        // signal back to main and wait until we're signaled to continue
        osSignalSet((osThreadId)mainID, MAIN_TASK_CONTINUE);
        Thread::signal_wait(SUB_TASK_CONTINUE, osWaitForever);

        // osThreadTerminate(threadID);
        // return;
    }

    // Wait until the threads with the CommModule class are all started up
    // and ready
    while (CommModule::isReady() == false) {
        Thread::wait(50);
    }

    // == everything below this line all the way until the start
    // of the while loop is test code ==
    const std::string linkTestMsg = "Hello World!";

    // Test RX acknowledgment with a packet structured to trigger the ACK
    // response
    rtp::packet ack_pck;
    ack_pck.header_link = RTP_HEADER(rtp::port::LINK, 1, false, false);
    ack_pck.payload_size = linkTestMsg.length() + 1;
    memcpy((char*)ack_pck.payload, linkTestMsg.c_str(), ack_pck.payload_size);
    // ack_pck.address = BASE_STATION_ADDR;
    ack_pck.address = LOOPBACK_ADDR;

    LOG(INIT,
        "Placing link test packet in RX buffer:\r\n"
        "\tPayload:\t%s\t(%u bytes)",
        (char*)ack_pck.payload, ack_pck.payload_size);

    CommModule::receive(ack_pck);

    // signal back to main and wait until we're signaled to continue
    osSignalSet((osThreadId)mainID, MAIN_TASK_CONTINUE);
    Thread::signal_wait(SUB_TASK_CONTINUE, osWaitForever);

    while (true) {
        Thread::wait(1500);
        Thread::yield();

        // CC1201 *should* fall into IDLE after it sends the packet. It will
        // then calibrate right before entering the RX state strobed below.
        // radio_900.strobe(CC1201_STROBE_SRX);
    }

    osThreadTerminate(threadID);
}
