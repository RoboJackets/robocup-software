#include "robot.hpp"

/**
 * [rx_callback This is executed for a successfully received radio packet.]
 * @param p [none]
 */
void rx_callback_test(RTP_t* p)
{
	if (p->payload_size > 0)
		LOG(OK, "%u byte packet received!", p->payload_size);
}

void Task_CommCtrl(void const* args)
{
	// Store the thread's ID
	osThreadId threadID = Thread::gettid();
	/*
		// Create a new physical hardware communication link
		CC1201 radio_900(
		    RJ_SPI_BUS,
		    RJ_RADIO_nCS,
		    RJ_RADIO_INT
		);

		CC1201Config *radioConfig = new CC1201Config();
		radioConfig = CC1201Config::resetConfiguration(radioConfig);
		CC1201Config::loadConfiguration(radioConfig, &radio_900);
		CC1201Config::verifyConfiguration(radioConfig, &radio_900);

		radio_900.freq();
		radio_900.set_rssi_offset(-81);

		// Create a Communication Module Object
		CommModule comm;	// don't think we need this anymore. may still be a few things that need changing tho

		// NO NEED FOR THIS ANYMORE ===
		// radio_900.setModule(comm);
		// ===

		CommModule::TxHandler((CommLink*)&radio_900, &CommLink::sendPacket, 8);
		CommModule::RxHandler(rx_callback_test, 8);
		CommModule::openSocket(8);

		// Create a dummy packet that is set to send out from socket connection 8
		RTP_t dummy_packet;

		dummy_packet.port = 8;
		dummy_packet.subclass = 1;
		dummy_packet.address = 255;
		dummy_packet.sfs = 0;
		dummy_packet.ack = 0;
		dummy_packet.payload_size = 25;

		// fill with 25 arbitrary bytes
		memset(dummy_packet.payload, 0xFF, 25);

		// CommModule::send(dummy_packet);	// As of now, the CC1201 should always be in RX and calibrated if necessary exiting this.

		LOG(OK, "Radio interface ready!");
	*/

	while (true) {
		Thread::yield();
		// CC1201 *should* fall into IDLE after it sends the packet. It will then calibrate right before entering the RX state strobed below.
		//radio_900.strobe(CC1201_STROBE_SRX);
	}

	osThreadTerminate(threadID);
}
