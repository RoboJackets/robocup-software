#include "robot.hpp"
#include "dma.hpp"
#include "adc-dma.hpp"


DigitalOut ledOne(LED1, 0);
DigitalOut is_locked(LED2, 0);
DigitalOut rssi_valid(LED3, 0);
DigitalOut led4(LED4, 1);
DigitalIn gpio3(p18);
DigitalIn gpio2(p16);
Serial pcserial(USBTX, USBRX);

ADCDMA adc;
DMA dma;

void initConsoleRoutine(void const *args);


/**
 * Timer interrupt based light flicker. If this stops, the code triggered
 * a fault.
 */
void imAlive(void const *args)
{
	DigitalOut *led = (DigitalOut *)args;

	*led = !(*led);
	osDelay(50);
	*led = !(*led);
}


/**
 * [rx_callback This is executed for a successfully received radio packet.]
 * @param p [none]
 */
void rx_callback(RTP_t *p)
{
	if (p->payload_size > 0)
		LOG(OK, "%u byte packet received!", p->payload_size);
}


/**
 * [main Main program.]
 * @return  [none]
 */
int main(void)
{
	is_locked = true;
	setISRPriorities();
	pcserial.baud(9600);

	RtosTimer led_1_ticker(imAlive, osTimerPeriodic, (void *)&ledOne);
	led_1_ticker.start(1000);

	RtosTimer RadioTimeoutTask(imAlive, osTimerPeriodic, (void *)&ledOne);

	isLogging = RJ_LOGGING_EN;
	rjLogLevel = INF1;

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
		//Watchdog::Set(RJ_WATCHDOG_TIMER_VALUE);

		while (1) {
			dummy_packet.port = 8;
			dummy_packet.subclass = 1;
			dummy_packet.address = 255;
			dummy_packet.sfs = 0;
			dummy_packet.ack = 0;
			dummy_packet.payload_size = 25;

			for (int i = 0; i < 25; i++)
				dummy_packet.payload[i] = 0x24;

			LOG(OK, "%u\r\n", comm.NumRXPackets());

			std::string current_state = decode_marcstate(radio_900.mode());
			LOG(OK, "  STATE: %s\tRSSI: %.1f dBm", current_state.c_str(), radio_900.rssi());

			// comm.send(dummy_packet);	// As of now, the CC1201 should always be in RX and calibrated if necessary exiting this.

			is_locked = radio_900.isLocked();
			rssi_valid = gpio2;

			osDelay(600);
			led4 = !led4;

			// CC1201 *should* fall into IDLE after it sends the packet. It will then calibrate right before entering the RX state strobed below.
			//radio_900.strobe(CC1201_STROBE_SRX);
		}
		*/

	uint32_t dma_locations[10] = { 0 };

	adc.AddChannel(RJ_BALL_DETECTOR);
	adc.AddChannel(RJ_BATT_SENSE);
	adc.AddChannel(RJ_5V_SENSE);

	if (adc.Start())
		LOG(INF1, "ADC config didn't break!");

	if (dma.Init()) {
		dma.SetDst((uint32_t) &dma_locations[0]);
		// dma.SetSrc((uint32_t) & (LPC_ADC->ADDR0));

		if (dma.Start())
			LOG(INF1, "DMA config didn't break!");
	}

	adc.BurstRead();
	led4 = 0;

	while (1) {
		LOG(INF2, "  0x%08X\r\n  0x%08X\r\n  0x%08X\r\n  ADGDR:\t0x%08X\r\n  ADINTEN:\t0x%08X\r\n  ADCR:\t\t0x%08X\r\n  Chan 0:\t0X%08X\r\n  Chan 1:\t0X%08X\r\n  Chan 2:\t0X%08X\r\n  INT Called:\t%s", dma_locations[0], dma_locations[1], dma_locations[2], LPC_ADC->ADGDR, LPC_ADC->ADINTEN, LPC_ADC->ADCR, LPC_ADC->ADDR0, LPC_ADC->ADDR1, LPC_ADC->ADDR2, (DMA::HandlerCalled ? "YES" : "NO"));
		LOG(INF2, "  DMACIntTCStat:\t0x%08X\r\n  DMACIntErrStat:\t0x%08X\r\n  DMACEnbldChns:\t0x%08X\r\n  DMACConfig:\t\t0x%08X", LPC_GPDMA->DMACIntTCStat, LPC_GPDMA->DMACIntErrStat, LPC_GPDMA->DMACEnbldChns, LPC_GPDMA->DMACConfig);
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
			osDelay(70);
			//ledOne = !ledOne;
		}

		//clear light for main loop (shows its complete)
		// ledTwo = false;

	} else {
		// return;
		while (true);
	}
}
