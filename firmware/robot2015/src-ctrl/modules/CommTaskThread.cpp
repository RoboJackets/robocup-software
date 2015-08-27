#include "pins-ctrl-2015.hpp"
#include "robot-config.hpp"
#include "TaskSignals.hpp"

#include <rtos.h>
#include <CommModule.hpp>
#include <CommPort.hpp>
#include <CC1201Radio.hpp>
#include <CC1201Config.hpp>
#include <logger.hpp>

/*
 * Information about the radio protocol can be found at:
 * https://www.overleaf.com/2187548nsfdps
 */

/*
* Example of sending a packet.

RTP_t pck;						// Declare a new packet structure

pck.port = 8;					// What port should the packet be routed to?
pck.subclass = 1;				// What subclass of the port is this for?
pck.ack = false;				// Do we need an acknowledgment or is this an acknowledgment response?
pck.sfs = false;				// Is the size field Significant? (Almost always 'No')

// The lines above can be set with the 'RTP_HEADER' macro as an alternative.
// RTP_HEADER(port, subclass, ack, sfs)
//
// pck.header_link = RTP_HEADER(8, 1, false, false);

pck.address = 255;				// Who are we sending the packet to? (255 is broadcast address)
pck.payload_size = 25;			// How many bytes are in the payload of the packet?

memset(pck.payload, 0xFF, pck.payload_size);	// fill with 25 arbitrary bytes

CommModule.send(pck);			// Send it!

// Note: A packet with a requested ACK must be accounted for within the RX callback function.
*/

static const uint8_t BASE_STATION_ADDR = 0x01;

#define ACK_RESPONSE_CODE 0xAA


/**
 * [rx_callback This is executed for a successfully received radio packet.]
 * @param p [none]
 */
void rxCallbackLinkTest(RTP_t* p)
{
	if (p->payload_size > 0) {
		LOG(INIT, "Radio RX working!.\r\n    Received %u byte packet.", p->payload_size);
	}
}


/**
 * [Task_CommCtrl description]
 * @param args [description]
 */
void Task_CommCtrl(void const* args)
{
	// Store the thread's ID
	osThreadId threadID = Thread::gettid();

	// Store our priority so we know what to reset it to if ever needed
	osPriority threadPriority;

	if (threadID != nullptr)
		threadPriority  = osThreadGetPriority(threadID);
	else
		threadPriority = osPriorityIdle;

	// Startup the CommModule interface
	CommModule::Init();

	// Create a new physical hardware communication link
	CC1201 radio(
	    RJ_SPI_BUS,
	    RJ_RADIO_nCS,
	    RJ_RADIO_INT
	);

	if (radio.isConnected() == true) {
		// Load the configuration onto the radio transceiver
		CC1201Config* radioConfig = new CC1201Config();
		radioConfig = CC1201Config::resetConfiguration(radioConfig);
		CC1201Config::loadConfiguration(radioConfig, &radio);
		CC1201Config::verifyConfiguration(radioConfig, &radio);

		LOG(INIT, "Radio interface ready on %3.2fMHz!\r\n    Thread ID:\t%u\r\n    Priority:\t%d", radio.freq(), threadID, threadPriority);

		/*
		// Open a socket for running tests across the link layer
		CommModule::RxHandler(&rxCallbackLinkTest, COMM_PORT_LINK_TEST);
		CommModule::TxHandler((CommLink*)&radio, &CommLink::sendPacket, COMM_PORT_LINK_TEST);
		CommModule::openSocket(COMM_PORT_LINK_TEST);

		// The usual way of opening a port.
		CommModule::RxHandler(&rxCallbackLinkTest, COMM_PORT_GAMEPLAY_STROBE);
		CommModule::TxHandler((CommLink*)&radio, &CommLink::sendPacket, COMM_PORT_GAMEPLAY_STROBE);
		CommModule::openSocket(COMM_PORT_GAMEPLAY_STROBE);		// returns true if port was successfully opened.

		// This port won't open since there's no RX callback to invoke. The packets are simply dropped.
		CommModule::RxHandler(&rxCallbackLinkTest, COMM_PORT_CONTROLLER);
		CommModule::TxHandler((CommLink*)&radio, &CommLink::sendPacket, COMM_PORT_CONTROLLER);
		CommModule::openSocket(COMM_PORT_CONTROLLER);

		// There's no TX callback for this port, but it will still open when invoked since it knows where to send an RX packet.
		CommModule::TxHandler((CommLink*)&radio, &CommLink::sendPacket, COMM_PORT_SETPOINT);
		CommModule::RxHandler(&rxCallbackLinkTest, COMM_PORT_SETPOINT);
		CommModule::openSocket(COMM_PORT_SETPOINT);
		*/

	} else {
		LOG(FATAL, "No radio interface found!\r\n    Terminating main radio thread.");
		// TODO: Turn on radio error LED here

		// Always keep the link test port open regardless
		// CommModule::RxHandler(nullptr, COMM_PORT_LINK_TEST);
		// CommModule::openSocket(COMM_PORT_LINK_TEST);

		osThreadTerminate(threadID);
		return;
	}

	/*
	 * Ports are always displayed in ascending (lowest -> highest) order according
	 * to its port number when using the console. Since most of everything is static,
	 * the CommModule methods can be used from almost anywhere.
	 */

	// == everything below this line all the way until the start of the while loop is test code ==

	char buf[] = "Hello World!";

	// Test RX by placing a dummy packet in the RX queue
	RTP_t pck;
	pck.header_link = RTP_HEADER(COMM_PORT_CONTROLLER, 1, false, false);
	pck.payload_size = 25;
	memset(pck.payload, 0xFF, pck.payload_size);
	pck.address = BASE_STATION_ADDR;

	// Test RX acknowledgment with a packet structured to trigger the ACK response
	RTP_t ack_pck;
	ack_pck.header_link = RTP_HEADER(COMM_PORT_LINK_TEST, 1, true, false);
	ack_pck.payload_size = sizeof(buf);
	memcpy(ack_pck.payload, buf, sizeof(buf));
	ack_pck.address = BASE_STATION_ADDR;

	LOG(INIT, "Placing dummy RX packet in radio buffer:\r\n    Payload:\t\"%s\"\t(%u bytes)", ack_pck.payload, ack_pck.payload_size);

	CommModule::receive(pck);

	while (true) {
		/*
		Thread::wait(100);

		// Simulate some incoming packets on 2 different ports
		//CommModule::receive(pck);

		//Thread::wait(1);

		// Now, simulate some incoming packets that sometimes request an ACK
		//CommModule::receive(ack_pck);

		CommModule::send(ack_pck);
		*/

		CommModule::receive(pck);
		Thread::wait(1500);
		Thread::yield();

		// CC1201 *should* fall into IDLE after it sends the packet. It will then calibrate right before entering the RX state strobed below.
		//radio_900.strobe(CC1201_STROBE_SRX);
	}

	osThreadTerminate(threadID);
}
