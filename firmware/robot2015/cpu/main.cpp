#include "robot.hpp"
#include "console.hpp"
#include "commands.hpp"
#include "logger.hpp"
#include "radio.hpp"

Ticker lifeLight;
DigitalOut ledOne(LED1);
DigitalOut ledTwo(LED2);

DigitalOut led(p20, 0);

/*
 * some forward declarations to make the code easier to read
 */
void imAlive(void);
void setISRPriorities(void);
void initRadioThread(void);
void initConsoleRoutine(void);

void writeByte(DigitalInOut*, char);
char readByte(DigitalInOut* pin);
unsigned int crc8_add(unsigned int acc, char b);

const int tREC = 5;
const int tSLOT = 65;
const int tRSTLmin = 480;
const int tRSTLmax = 640;
const int tRSTLavg = 540;
const int tPDHmin = 15;
const int tPDHmax = 60;
const int tPDHavg = 35;
const int tPDLmin = 60;
const int tPDLmax = 240;
const int tPDLavg = 150;
const int tFPD = 8;
const int tMSPmin = 60;
const int tMSPmax = 75;
const int tMSPavg = 68;
const int tRSTH = 480;
const int tW0Lmin = 60;
const int tW0Lmax = 120;
const int tW0Lavg = 90;
const int tW1Lmin = 5;
const int tW1Lmax = 15;
const int tW1Lavg = 10;
const int tRLmin = 5;
const int tRLmax = 15;
const int tRLavg = 6;
const int tMSR = 15;

/**
 * system entry point
 */
int main(void) 
{
	setISRPriorities();
	lifeLight.attach(&imAlive, 0.25);

	DigitalInOut idPin(p21, PIN_OUTPUT, PullUp, 1);

	printf("\033[2J");
	printf("Communicating with ID Chip...\r\n");

	// Reset signal, low for 480us
	idPin = 0;
	wait_us(tRSTLavg); // Trstl
	idPin = 1;

	// wait for presence response
	wait_us(tMSPavg); // Tmsp
	idPin.input();
	if(idPin == 0) {
		wait_us(tRSTH - tMSPavg); // wait for rest of tRSTH

		writeByte(&idPin, 0x33);

		char family = readByte(&idPin);

		char serial[6];
		for(int i = 0; i < 6; i++)
			serial[i] = readByte(&idPin);

		char crc = readByte(&idPin);

		printf("Family byte  : 0x%x\r\n", family);
		
		printf("Serial byte  : 0x");
		for(int i = 5; i >= 0; i--)
			printf("%x", serial[i]);

		printf("\r\nCRC          : 0x%x \r\n", crc);

		unsigned int calcCRC = crc8_add(0x0, family);
		for(int i = 0; i < 6; i++)
			calcCRC = crc8_add(calcCRC, serial[i]);

		printf("CRCs match?  : %d\r\n", calcCRC == crc);

		for(int i = 0; i < 17; i++)
			printf("\n");

		while(1) {
			led = !led;
			wait(1.0);
		}
	}
	else {
		printf("Failure :(\r\n");
		while(1) {
			led = !led;
			wait(0.5);
		}
	}

	isLogging = true;
	rjLogLevel = INF2;
	
	//initRadioThread();
	initConsoleRoutine();
}

void writeOne(DigitalInOut* pin) {
	*pin = 0;
	wait_us(tW1Lavg);
	*pin = 1;
	wait_us(tSLOT);
}

void writeZero(DigitalInOut* pin) {
	*pin = 0;
	wait_us(tW0Lavg);
	*pin = 1;
	wait_us(tREC);
}

void writeByte(DigitalInOut* pin, char b) {
	pin->output();

	for(uint i = 1; i < 256; i <<= 1) {
		if((b & i) == i)
			writeOne(pin);
		else
			writeZero(pin);
	}

	printf("Sent         : 0x%x\r\n", b);
}

char readByte(DigitalInOut* pin) {
	char value = 0;
	for(int i = 0; i < 8; i++) {
		pin->output();
		*pin = 0;
		wait_us(tRLavg);
		*pin = 1;

		wait_us(tMSR - tRLavg);
		pin->input();

		int bit = *pin;
		value |= bit << i;

		wait_us(tSLOT - tMSR);
	}

	return value;
}

unsigned int crc8_add(unsigned int acc, char byte) {
	acc ^= byte;

	for(int i = 0; i < 8; i++) {
		if(acc & 1) {
			acc = (acc >> 1) ^ 0x8c;
		}
		else {
			acc >>= 1;
		}
	}

	return acc;
}

/**
 * Initializes the peripheral nested vector interrupt controller (PNVIC) with 
 * appropriate values. Low values have the higest priority (with system 
 * interrupts having negative priority). All maskable interrupts are
 * diabled; do not intialize any interrupt frameworks before this funtion is
 * called. PNVIC interrupt priorities are independent of thread and NVIC 
 * priorities.
 * 
 * The PNVIC is independent of the NVIC responsible for system NMI's. The NVIC
 * is not accissable through the library, so in this context the NVIC functions
 * refer to the PNVIC. The configuration system for PNVIC priorities is strange
 * and different from X86. If you haven't already, look over 
 * doc/ARM-Cortex-M_Interrupt-Priorities.pdf from RJ root and the online 
 * documentation regarding Interrupt Priority Registers (IPRs) first. 
 */
void setISRPriorities(void)
{
	//set two bits for preemptive data, two bits for priority as the
	//structure for the IPRs. Grouping is global withing the IPRs so
	//this value should only be changed here.
	NVIC_SetPriorityGrouping(5);
	uint32_t priorityGrouping = NVIC_GetPriorityGrouping();

	//set preemptive priority default to 2 (0..3)
	//set priority default to 1 (0..3)
	uint32_t defaultPriority = NVIC_EncodePriority(priorityGrouping, 2, 1);

	//When the kernel initialzes the PNVIC, all ISRs are set to the
	//highest priority, making it impossible to elevate a few over
	//the rest, so the default priority is lowered globally for the
	//table first.
	//
	//Consult LPC17xx.h under IRQn_Type for PNVIC ranges, this is LPC1768 
	//specific
	for (uint32_t IRQn = TIMER0_IRQn; IRQn <= CANActivity_IRQn; IRQn++)
	{
		//set default priority
		NVIC_SetPriority((IRQn_Type) IRQn, defaultPriority);
	}

	////////////////////////////////////
	//  begin raise priority section  //
	////////////////////////////////////

	//reestablish watchdog
	NVIC_SetPriority(WDT_IRQn, NVIC_EncodePriority(priorityGrouping, 0, 0));

	//TODO raise radio
	//TODO raise others after discussion

	////////////////////////////////////
	//  begin lower priotity section  //
	////////////////////////////////////

	//set UART (console) interrupts to minimal priority
	//when debugging radio and other time sensitive operations, this
	//interrupt will need to be deferred, especially given the number of
	//calls to printf()
	NVIC_SetPriority(UART0_IRQn, NVIC_EncodePriority(priorityGrouping, 3, 0));
	NVIC_SetPriority(UART1_IRQn, NVIC_EncodePriority(priorityGrouping, 3, 2));
	NVIC_SetPriority(UART2_IRQn, NVIC_EncodePriority(priorityGrouping, 3, 2));
	NVIC_SetPriority(UART3_IRQn, NVIC_EncodePriority(priorityGrouping, 3, 1));

	//TODO lower others after discussion

	//NVIC_EnableIRQ(TIMER0_IRQn);
	//NVIC_EnableIRQ(TIMER1_IRQn);
	//NVIC_EnableIRQ(TIMER2_IRQn);
	//NVIC_EnableIRQ(TIMER3_IRQn);
}

/**
 * initializes the radio communications thread
 */
void initRadioThread(void)
{
	initRadio();
}

/**
 * initializes the console
 */
void initConsoleRoutine(void)
{
	if (!COMPETITION_DEPLOY)
	{
		initConsole();

		for (;;)
		{
			//check console communications, currently does nothing
			//then execute any active iterative command
			conComCheck();
			//execute any active iterative command
			executeIterativeCommand();

			//check if a system stop is requested
			if (isSysStopReq() == true)
			{
				break;
			}

			//main loop heartbeat
			wait(0.1);
			ledTwo = !ledTwo;
	    	}

		//clear light for main loop (shows its complete)
		ledTwo = false;
	}
	else
	{
		for (;;);
	}
}

/**
 * timer interrupt based light flicker. If this stops, the code triggered
 * a fault.
 */
void imAlive()
{
	ledOne = !ledOne;
}
