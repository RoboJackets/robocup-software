

#include "robot.hpp"
#include "console.hpp"
#include "radio-state-decode.hpp"
#include "isr-prop.hpp"
#include "rtos.h"
#include "dma.hpp"
#include "adc-dma.hpp"


Ticker lifeLight;
DigitalOut ledOne(LED1, 1);
DigitalOut is_locked(LED2, 0);
DigitalOut rssi_valid(LED3, 0);
DigitalOut led4(LED4, 1);
DigitalIn gpio3(p18);
DigitalIn gpio2(p16);
//Serial pcserial(USBTX, USBRX);

ADCDMA adc;
DMA dma;


void initConsoleRoutine(void const *args);

/**
 * Timer interrupt based light flicker. If this stops, the code triggered
 * a fault.
 */
void imAlive(void)
{
	ledOne = !ledOne;
}


/**
 * [rx_callback This is executed for a successfully received radio packet.]
 * @param p [none]
 */
void rx_callback(RTP_t *p)
{
	if (p->payload_size > 0)
		log(OK, "main.cpp", "%u byte packet received!", p->payload_size);
}


/**
 * [main Main program.]
 * @return  [none]
 */
int main(void)
{
	led4 = false;

	// pcserial.baud(9600);

	//setISRPriorities();

	lifeLight.attach(&imAlive, 0.25);

	isLogging = true;
	rjLogLevel = OK;

	Thread taskConsole(initConsoleRoutine);
	/*
		// led4 = true;

		// Create a new physical hardware communication link
		CC1201 radio_900(
		    RJ_SPI_BUS,
		    RJ_RADIO_nCS,
		    RJ_RADIO_INT
		);

		led4 = true;

		CC1201Config *radioConfig = new CC1201Config();
		radioConfig = CC1201Config::resetConfiguration(radioConfig);
		CC1201Config::loadConfiguration(radioConfig, &radio_900);
		CC1201Config::verifyConfiguration(radioConfig, &radio_900);

		radio_900.freq();
		radio_900.set_rssi_offset(-81);

		// Create a Communication Module Object
		CommModule comm;
		radio_900.setModule(comm);

		comm.TxHandler((CommLink *)&radio_900, &CommLink::sendPacket, 8);
		comm.RxHandler(rx_callback, 8);
		comm.openSocket(8);

		// Create a dummy packet that is set to send out from socket connection 8
		// RTP_t dummy_packet;

		// Enable watchdog timer
		//Watchdog watchdog;
		//watchdog.set(RJ_WATCHDOG_TIMER_VALUE);

		while (1) {
			dummy_packet.port = 8;
			dummy_packet.subclass = 1;
			dummy_packet.address = 255;
			dummy_packet.sfs = 0;
			dummy_packet.ack = 0;
			dummy_packet.payload_size = 25;

			for (int i = 0; i < 25; i++)
				dummy_packet.payload[i] = 0x24;

			log(OK, "MAIN", "%u\r\n", comm.NumRXPackets());

			std::string current_state = decode_marcstate(radio_900.mode());
			log(OK, "MAIN", "  STATE: %s\tRSSI: %.1f dBm", current_state.c_str(), radio_900.rssi());

			// comm.send(dummy_packet);	// As of now, the CC1201 should always be in RX and calibrated if necessary exiting this.

			is_locked = radio_900.isLocked();
			rssi_valid = gpio2;

			osDelay(600);
			led4 = !led4;

			// CC1201 *should* fall into IDLE after it sends the packet. It will then calibrate right before entering the RX state strobed below.
			//radio_900.strobe(CC1201_STROBE_SRX);
		}
		*/

	uint32_t adc_vals[10] = { 0 };

	dma.setDst(NULL, adc_vals);
	ledOne = false;

	//dma.start();
	//adc.BurstRead();

	while (1) {
		osDelay(1000);
	}

}


/**
 * initializes the console
 */
void initConsoleRoutine(void const *args)
{
	if (!COMPETITION_DEPLOY) {
		Console::Init();

		while (true) {
			//check console communications, currently does nothing
			//then execute any active iterative command
			Console::ConComCheck();
			//execute any active iterative command
			executeIterativeCommand();

			//check if a system stop is requested
			if (Console::IsSystemStopRequested() == true)
				break;

			//main loop heartbeat
			osDelay(250);
			ledOne != ledOne;
		}

		//clear light for main loop (shows its complete)
		// ledTwo = false;

	} else {
		// return;
		while (true);
	}
}
