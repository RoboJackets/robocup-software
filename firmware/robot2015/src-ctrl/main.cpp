#include "robot.hpp"

#include <cstdarg>
#include <ctime>

#include <logger.hpp>

#include "commands.hpp"
//#include "mcp23017.hpp"
#include "io-expander.hpp"
// #include "neostrip.cpp"

// ADCDMA adc;
// DMA dma;

// Sets the correct hardware configurations for pins connected to LEDs
void statusLights(bool state)
{
	DigitalInOut init_leds[4] = {
		{RJ_BALL_LED, PIN_OUTPUT, OpenDrain, state},
		{RJ_RDY_LED, PIN_OUTPUT, OpenDrain, state},
		{RJ_RX_LED, PIN_OUTPUT, OpenDrain, state},
		{RJ_TX_LED, PIN_OUTPUT, OpenDrain, state}
	};

	for (int i = 0; i < 4; i++)
		init_leds[i].mode(PullUp);

	// Keeps the `ready` LED on after the others are turned off
	if (state == 1)
		init_leds[1] = 0;
}

void statusLightsON(void const* args)
{
	statusLights(0);
}

void statusLightsOFF(void const* args)
{
	statusLights(1);
}

/**
 * [main Main The entry point of the system where each submodule's thread is started.]
 * @return  [none]
 */
int main(void)
{
	// Turn on some startup LEDs to show they're working
	statusLightsON(nullptr);

	// Set the default logging configurations
	isLogging = RJ_LOGGING_EN;
	rjLogLevel = INIT;

	/* Always send out an empty line at startup for keeping the console
	 * clean on after a 'reboot' command is called;
	 */
	if (isLogging) {
		printf("\r\n\r\n");
		fflush(stdout);
	}

	// MCP23017::Init();

	/* Set the system time to the most recently known time. The compilation
	 * time is used here. The timestamp should always be the same when using GCC.
	 */
	/*
	const char* sysTime = __DATE__ " " __TIME__;
	std::tm* tt;

	if (std::strftime(sysTime, sizeof(sysTime),"%b %d %Y %H:%M:%S", tt) == 0) {	// 'Mmm DD YYYYHH:MM:SS'
		LOG(SEVERE, "Unable to parse system time of %s", sysTime);
	} else {
		set_time(mktime(tt));
	}
	*/

	// Setup the interrupt priorities before launching each subsystem's task thread.
	setISRPriorities();

	// Start a periodic blinking LED to show system activity
	DigitalOut ledOne(LED1, 0);
	RtosTimer live_light(imAlive, osTimerPeriodic, (void*)&ledOne);
	live_light.start(RJ_LIFELIGHT_TIMEOUT_MS);

	// TODO: write a function that will recalibrate the radio for this.
	// Reset the ticker on every received packet. For now, we just blink an LED.
	DigitalOut led4(LED4, 0);
	RtosTimer radio_timeout_task(imAlive, osTimerPeriodic, (void*)&led4);
	radio_timeout_task.start(300);

	// Flip off the startup LEDs after a timeout period
	RtosTimer init_leds_off(statusLightsOFF, osTimerOnce);

	MCP23017::Init();

	// Setup some extended LEDs and turn them on
	IOExpanderDigitalOut led_err_m1(IOExpanderPinB0);

	uint8_t robot_id = MCP23017::digitalWordRead() & 0x0F;
	LOG(INIT, "Robot ID:\r\n    %u", robot_id);

	led_err_m1 = 1;

	LOG(INIT, "    0x%04X", MCP23017::read_mask(0xFFFF));

	for (int i = 0; i < 0x14; i++)
		LOG(INIT, "    Addr:\t0x%04X\r\n    Val:\t0x%04X", i, MCP23017::readRegister(i));

	motors_Init();

	// Wait for anything before now to print things out of the serial port.
	// That way, things should stay lined up in the console when starting all the threads.
	Thread::wait(1000);

	// Start the thread task for the on-board control loop
	Thread controller_task(Task_Controller, NULL, osPriorityRealtime);

	// Start the thread task for handling radio communications
	Thread comm_task(Task_CommCtrl, NULL, osPriorityHigh);

	// Start the thread task for the serial console
	Thread console_task(Task_SerialConsole, NULL, osPriorityBelowNormal);

	//FPGA fpga;

	//if (fpga.Init() == false)
	//	LOG(FATAL, "FPGA config failed!");


#ifdef LINK_TOC_PARAMS
	TOCInit();

	// Start the thread task for setting up a dynamic logging structure
	MailHelper<RTP_t, 5> tocQueue;

	osMailQId tocQID = osMailCreate(tocQueue.def(), NULL);

	Thread toc_task(Task_TOC, &tocQID, osPriorityBelowNormal);


	// Start the thread task for access to certain variables in the dynamic logging structure
	MailHelper<RTP_t, 5> paramQueue;

	osMailQId paramQID = osMailCreate(paramQueue.def(), NULL);

	Thread param_task(Task_Param, &paramQID, osPriorityBelowNormal);

#endif


	/*
	NeoStrip rgbLED(p21, 1);
	rgbLED.setBrightness(1.0);
	rgbLED.setPixel(0, 0x00, 0xFF, 0x00);
	rgbLED.write();
	*/


#if RJ_WATCHDOG_TIMER_EN
	// Enable the watchdog timer if it's set in configurations.
	Watchdog::Set(RJ_WATCHDOG_TIMER_VALUE);

#endif


	/* This needs some work. Probably best to just drop DMA for the ADC and
	 * configure the 3 ADC channels at startup to be in burst mode at a very
	 * low rate. Then the ADC registers should always have a valid reading.
	 */

	/*
	adc.SetChannels({ RJ_BALL_DETECTOR, RJ_BATT_SENSE, RJ_5V_SENSE });

	if (!adc.Start()) {
		DigitalOut led3(LED3, 1);

		// error
	}

	if (!dma.Init()) {	// currently hard coded to always return false
		DigitalOut led2(LED4, 1);

		// error
	}

	adc.BurstRead();
	*/

	/*
		RTP_t* paramBlk = (RTP_t*)osMailAlloc(paramQID, 1000);
		paramBlk->subclass = 1;	// read channel
		paramBlk->payload[0] = 1;	// read channel
		osMailPut(paramQID, paramBlk);
	*/


	/*
	 * Uncomment this block to show info about the bit packing of a radio packet
	 *
	RTP_t pkt;
	pkt.header_link = RTP_HEADER(0x0F, 0x00, true, true);
	LOG(INIT,
	    "Header:\t\t0x%02X\r\n"
	    "Port:\t\t%u\r\n"
	    "Subclass:\t%u\r\n"
	    "ACK:\t\t%u\r\n"
	    "SFS:\t\t%u",
	    pkt.header_link,
	    pkt.port,
	    pkt.subclass,
	    pkt.ack,
	    pkt.sfs
	   );
	*/

	init_leds_off.start(1000);

	while (1) {
		//LOG(INF3, "  0x%08X\r\n  0x%08X\r\n  0x%08X\r\n  ADGDR:\t0x%08X\r\n  ADINTEN:\t0x%08X\r\n  ADCR:\t\t0x%08X\r\n  Chan 0:\t0X%08X\r\n  Chan 1:\t0X%08X\r\n  Chan 2:\t0X%08X\r\n  INT Called:\t%s", dma_locations[0], dma_locations[1], dma_locations[2], LPC_ADC->ADGDR, LPC_ADC->ADINTEN, LPC_ADC->ADCR, LPC_ADC->ADDR0, LPC_ADC->ADDR1, LPC_ADC->ADDR2, (DMA::HandlerCalled ? "YES" : "NO"));
		//LOG(INF3, "  DMACIntTCStat:\t0x%08X\r\n  DMACIntErrStat:\t0x%08X\r\n  DMACEnbldChns:\t0x%08X\r\n  DMACConfig:\t\t0x%08X", LPC_GPDMA->DMACIntTCStat, LPC_GPDMA->DMACIntErrStat, LPC_GPDMA->DMACEnbldChns, LPC_GPDMA->DMACConfig);

		Thread::wait(1000);	// Ping back to main every 1 second seems to perform better than calling Thread::yeild() for some reason?
	}
}


// The below commented code are things that I worked towards but never brought to a functional state

/*
// Power up timer 0
LPC_SC->PCONP |= (1 << 1);

// Set divider to CLK/1
LPC_SC->PCLK0 |= (0x01 << 2);

// LPC_PINCON->PINSEL4 |= (0x00);
//
PINCON->PINMODE4 |= (0x03);

// select timer pins in PINSEL reg
//select pin modes in PINMODE
// Interrupt set enable register for interrupt

NVIC_SetVector(TIMER0_IRQn, (uint32_t)TIMER0_IRQHandler);
NVIC_EnableIRQ(TIMER0_IRQn);
*/

/*
// Start the flash signature generation
*(long unsigned int*)0x40084024 |= (0x01 << 17) | ((LPC_RAM_BASE / 16) << 16);

// Wait for the flash signature to be generated
while ( !((*(long unsigned int*)0x40084FE0) & (0x01 << 2)) ) {};

LOG(OK,
    "Flash Signature: 0x%08X-%08X-%08X-%08X",
    *(long unsigned int*)0x4008402C,
    *(long unsigned int*)0x40084030,
    *(long unsigned int*)0x40084034,
    *(long unsigned int*)0x40084038
   );
*/
