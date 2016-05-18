/**
 * This program sends a robot status packet whenever it receives a control
 * message addressed to it (robot id 1).  It prints a '-->' to the console for
 * every packet sent and a '<--' for every packet received. This program was
 * created to test the cc1201 configuration/driver in order to ensure that
 * everything works and to tune the register settings.  It is meant to be used
 * along with the radio-sender-test program.
 */

#include <cmsis_os.h>
#include <mbed.h>
#include <memory>

#include "CC1201Radio.hpp"
#include "SharedSPI.hpp"
#include "logger.hpp"
#include "pins-ctrl-2015.hpp"

using namespace std;

const uint8_t ROBOT_UID = 1;

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

void radioRxHandler(rtp::packet* pkt) {
    printf("<--\r\n");

    rtp::ControlMessage controlMsg;
    bool success = rtp::DeserializeFromBuffer(&controlMsg, pkt->payload.data(),
                                              pkt->payload.size());
    if (!success) {
        printf("bad rx\r\n");
        return;
    }

    if (controlMsg.uid != ROBOT_UID) {
        printf("not addressed by packet\r\n");
        return;
    }

    // send reply packet
    rtp::packet replyPkt;
    replyPkt.header.port = rtp::Port::CONTROL;
    replyPkt.header.address = rtp::BROADCAST_ADDRESS;

    // create control message and add it to the packet payload
    rtp::RobotStatusMessage msg;
    msg.uid = 1;
    msg.battVoltage = 12;
    rtp::SerializeToVector(msg, &replyPkt.payload);

    // transmit!
    CommModule::Instance->send(replyPkt);
    printf("-->\r\n");
}

int main() {
    // set baud rate to higher value than the default for faster terminal
    Serial s(RJ_SERIAL_RXTX);
    s.baud(57600);

    // Set the default logging configurations
    isLogging = RJ_LOGGING_EN;
    rjLogLevel = INIT;

    printf("****************************************\r\n");
    LOG(INIT, "Radio test receiver starting...");

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

    DigitalOut radioStatusLed(LED4, global_radio->isConnected());

    // wait for incoming packets
    while (true) {
        Thread::yield();
    }
}
