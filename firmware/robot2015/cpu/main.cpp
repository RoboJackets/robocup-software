#include "robot.hpp"
#include "motors.hpp"
#include "neopixel.hpp"
#include "controller.hpp"

DigitalOut ledOne(LED1, 0);
DigitalOut led4(LED4, 0);
DigitalIn gpio3(p18);
DigitalIn gpio2(p16);
ADCDMA adc;
DMA dma;

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
 * [main Main program.]
 * @return  [none]
 */
int main(void)
{
	isLogging = RJ_LOGGING_EN;
	rjLogLevel = INF1;

	setISRPriorities();

	// Start a periodic blinking LED to show system activity
	RtosTimer live_light(imAlive, osTimerPeriodic, (void *)&ledOne);
	live_light.start(1300);

	// TODO: write a function that will recalibrate the radio for this.
	// Reset the ticker on every received packet. For now, we just blink an LED.
	RtosTimer radio_timeout_task(imAlive, osTimerPeriodic, (void *)&led4);
	radio_timeout_task.start(300);

	// Start the thread task for the serial console
	Thread console_task(Task_SerialConsole, NULL, osPriorityBelowNormal);

	motors_Init();

	// Create a temporary DigitalIn so we can configure the pull-down resistor.
	// (The mbed API doesn't provide any other way to do this.)
	// An alternative is to connect an external pull-down resistor.
	// DigitalIn(p21, PullDown);

	// The pixel array control class.
	/*
	neopixel::PixelArray rebLED(p21);

	neopixel::Pixel p[1];
	p[0].red = 0x00;
	p[0].green = 0x00;
	p[0].blue = 0xFF;
	rebLED.update(p, 1);
	*/

	// This breaks everything
	// Thread comm_task(Task_CommCtrl, NULL, osPriorityNormal);

	// Launch the motion controller thread
	Thread controller_task(Task_Controller, NULL, osPriorityNormal);

	// Enable watchdog timer
	// Watchdog::Set(RJ_WATCHDOG_TIMER_VALUE);

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

	while (1) {
		LOG(INF2, "  0x%08X\r\n  0x%08X\r\n  0x%08X\r\n  ADGDR:\t0x%08X\r\n  ADINTEN:\t0x%08X\r\n  ADCR:\t\t0x%08X\r\n  Chan 0:\t0X%08X\r\n  Chan 1:\t0X%08X\r\n  Chan 2:\t0X%08X\r\n  INT Called:\t%s", dma_locations[0], dma_locations[1], dma_locations[2], LPC_ADC->ADGDR, LPC_ADC->ADINTEN, LPC_ADC->ADCR, LPC_ADC->ADDR0, LPC_ADC->ADDR1, LPC_ADC->ADDR2, (DMA::HandlerCalled ? "YES" : "NO"));
		LOG(INF2, "  DMACIntTCStat:\t0x%08X\r\n  DMACIntErrStat:\t0x%08X\r\n  DMACEnbldChns:\t0x%08X\r\n  DMACConfig:\t\t0x%08X", LPC_GPDMA->DMACIntTCStat, LPC_GPDMA->DMACIntErrStat, LPC_GPDMA->DMACEnbldChns, LPC_GPDMA->DMACConfig);
		osDelay(1000);
	}
}

