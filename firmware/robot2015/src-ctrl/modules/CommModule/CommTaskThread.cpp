#include "pins-ctrl-2015.hpp"
#include "CC1201Radio.hpp"
#include "CommModule.hpp"
#include "rtos.h"

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


enum PORTS {
	COMM_PORT_LINK_TEST 		= 0x01,
	COMM_PORT_CONTROLLER 		= 0x02,
	COMM_PORT_SETPOINT 			= 0x03,
	COMM_PORT_GAMEPLAY_STROBE 	= 0x04,
	COMM_PORT_DISCOVERY 		= 0x05,
	COMM_PORT_BULK_DATA 		= 0x06
};


/**
 * [rx_callback This is executed for a successfully received radio packet.]
 * @param p [none]
 */
void rxCallbackLinkTest(RTP_t* p)
{
	if (p->payload_size > 0) {
		LOG(OK, "%u byte packet received!", p->payload_size);
	}

	// Send back an ACK if we need
	if (p->ack == true) {

		// Make sure this isn't an ACK for one of our own packets first
		if (p->payload_size != 1 && (p->payload[0] != ACK_RESPONSE_CODE)) {
			RTP_t res;

			res.header_link = p->header_link;
			res.payload[0] = ACK_RESPONSE_CODE;
			res.payload_size = 1;
			res.address = BASE_STATION_ADDR;

			CommModule::send(res);
		}
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

	// Setup the TX/RX lights and have them off initially
	DigitalOut txLED(RJ_TX_LED, 1);
	DigitalOut rxLED(RJ_RX_LED, 1);

	// Create a new physical hardware communication link
	CC1201 radio(
	    RJ_SPI_BUS,
	    RJ_RADIO_nCS,
	    RJ_RADIO_INT
	);

	/*
	CC1201Config* radioConfig = new CC1201Config();
	radioConfig = CC1201Config::resetConfiguration(radioConfig);
	CC1201Config::loadConfiguration(radioConfig, &radio_900);
	CC1201Config::verifyConfiguration(radioConfig, &radio_900);
	*/

	// Update the frequency offset
	radio.freqUpdate();

	LOG(INF1, "Radio set for %.2fMHz", radio.freq());

	// Open a socket for running tests across the link layer
	CommModule::TxHandler((CommLink*)&radio, &CommLink::sendPacket, COMM_PORT_LINK_TEST);
	CommModule::RxHandler(&rxCallbackLinkTest, COMM_PORT_LINK_TEST);

	if ( CommModule::openSocket(COMM_PORT_LINK_TEST) )
		txLED = 1;

	CommModule::TxHandler((CommLink*)&radio, &CommLink::sendPacket, COMM_PORT_CONTROLLER);
	CommModule::RxHandler(&rxCallbackLinkTest, COMM_PORT_CONTROLLER);
	//CommModule::openSocket(COMM_PORT_CONTROLLER);

	CommModule::TxHandler((CommLink*)&radio, &CommLink::sendPacket, COMM_PORT_SETPOINT);
	//CommModule::RxHandler(&rxCallbackLinkTest, COMM_PORT_SETPOINT);
	CommModule::openSocket(COMM_PORT_SETPOINT);

	//CommModule::TxHandler((CommLink*)&radio, &CommLink::sendPacket, COMM_PORT_GAMEPLAY_STROBE);
	CommModule::RxHandler(&rxCallbackLinkTest, COMM_PORT_GAMEPLAY_STROBE);
	CommModule::openSocket(COMM_PORT_GAMEPLAY_STROBE);

	LOG(OK, "Radio interface ready! Thread ID: %u", threadID);

	// Turn off the TX/RX LEDs once the hardware is ready and ports are setup.
	//txLED = 0;
	//rxLED = 0;

	// Test RX by placing a dummy packet in the RX queue
	RTP_t pck;
	pck.header_link = RTP_HEADER(8, 1, false, false);
	pck.address = 255;
	pck.payload_size = 25;
	memset(pck.payload, 0xFF, pck.payload_size);

	//CommModule::receive(pck);
	//CommModule::send(pck);

	while (1) {
		Thread::wait(200);
		//Thread::yield();
		CommModule::receive(pck);

		// CC1201 *should* fall into IDLE after it sends the packet. It will then calibrate right before entering the RX state strobed below.
		//radio_900.strobe(CC1201_STROBE_SRX);
	}

	osThreadTerminate(threadID);
}
