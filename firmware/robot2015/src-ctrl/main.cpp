#include "robot.hpp"

#include <ctime>

#include "controller.hpp"
#include "MailHelper.hpp"
#include "CommModule.hpp"
// #include "neostrip.cpp"

DigitalIn gpio3(p18);
DigitalIn gpio2(p16);
ADCDMA adc;
DMA dma;


extern "C" void TIMER0_IRQHandler(void)
{

}

/**
 * Timer interrupt based light flicker. If this stops, the code triggered
 * a fault.
 */
void imAlive(void const* args)
{
	DigitalOut* led = (DigitalOut*)args;

	*led = !(*led);
	osDelay(50);
	*led = !(*led);
}

/**
 * [main Main program.]
 * @return  [none]
 */
int main(void)
{
	isLogging = RJ_LOGGING_EN;
	rjLogLevel = INF1;

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


	// Set the system time to the build time
	const char* sysTime = __DATE__ " " __TIME__;
	struct tm tt;

	if (strptime(sysTime, "%b %d %Y %H:%M:%S", &tt) == 0) // 'Mmm DD YYYYHH:MM:SS'
		LOG(SEVERE, "Unable to parse system time of %s", sysTime);

	set_time(mktime(&tt));

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

	setISRPriorities();

	// Start a periodic blinking LED to show system activity
	DigitalOut ledOne(LED1, 0);

	RtosTimer live_light(imAlive, osTimerPeriodic, (void*)&ledOne);

	live_light.start(1300);

	// TODO: write a function that will recalibrate the radio for this.
	// Reset the ticker on every received packet. For now, we just blink an LED.
	DigitalOut led4(LED4, 0);

	RtosTimer radio_timeout_task(imAlive, osTimerPeriodic, (void*)&led4);

	radio_timeout_task.start(300);

	// Start the thread task for the serial console
	Thread console_task(Task_SerialConsole, NULL, osPriorityLow);

	motors_Init();

	// This breaks everything
	Thread comm_task(Task_CommCtrl, NULL, osPriorityHigh);

	// Launch the motion controller thread
	Thread controller_task(Task_Controller, NULL, osPriorityRealtime);

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
	// Enable watchdog timer
	Watchdog::Set(RJ_WATCHDOG_TIMER_VALUE);

#endif

	uint32_t dma_locations[10] = { 0 };

	adc.AddChannel(RJ_BALL_DETECTOR);

	adc.AddChannel(RJ_BATT_SENSE);

	adc.AddChannel(RJ_5V_SENSE);

	if (adc.Start())
		LOG(INF1, "ADC config didn't break!");

	if (dma.Init()) {
		dma.SetDst((uint32_t) dma_locations);
		dma.SetSrc((uint32_t) & (LPC_ADC->ADDR0));

		if (dma.Start())
			LOG(INF1, "DMA config didn't break!");
	}

	adc.BurstRead();

	/*
		RTP_t* paramBlk = (RTP_t*)osMailAlloc(paramQID, 1000);
		paramBlk->subclass = 1;	// read channel
		paramBlk->payload[0] = 1;	// read channel
		osMailPut(paramQID, paramBlk);
	*/

	while (1) {
		LOG(INF3, "  0x%08X\r\n  0x%08X\r\n  0x%08X\r\n  ADGDR:\t0x%08X\r\n  ADINTEN:\t0x%08X\r\n  ADCR:\t\t0x%08X\r\n  Chan 0:\t0X%08X\r\n  Chan 1:\t0X%08X\r\n  Chan 2:\t0X%08X\r\n  INT Called:\t%s", dma_locations[0], dma_locations[1], dma_locations[2], LPC_ADC->ADGDR, LPC_ADC->ADINTEN, LPC_ADC->ADCR, LPC_ADC->ADDR0, LPC_ADC->ADDR1, LPC_ADC->ADDR2, (DMA::HandlerCalled ? "YES" : "NO"));
		LOG(INF3, "  DMACIntTCStat:\t0x%08X\r\n  DMACIntErrStat:\t0x%08X\r\n  DMACEnbldChns:\t0x%08X\r\n  DMACConfig:\t\t0x%08X", LPC_GPDMA->DMACIntTCStat, LPC_GPDMA->DMACIntErrStat, LPC_GPDMA->DMACEnbldChns, LPC_GPDMA->DMACConfig);
		Thread::wait(1000);
	}
}

