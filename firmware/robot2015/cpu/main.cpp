#include "robot.hpp"
#include "console.hpp"
#include "commands.hpp"
#include "logger.hpp"
#include "CC1201Radio.hpp"

Ticker lifeLight;
DigitalOut ledOne(LED1);
DigitalOut is_locked(LED2, 0);
DigitalOut rssi_valid(LED3, 0);
DigitalOut led4(LED4, 0);
DigitalIn gpio3(p18);
DigitalIn gpio2(p16);
Serial pcserial(USBTX, USBRX);

/*
 * some forward declarations to make the code easier to read
 */
 void imAlive(void);
 void setISRPriorities(void);
 void initRadioThread(void);
 void initConsoleRoutine(void);

 void rx_callback(RTP_t* p)
 {
 	log(OK, "main.cpp", "%u byte packet received!", p->payload_size);
 }


 int main(void) 
 {
 	pcserial.baud(57600);
 	setISRPriorities();
 	lifeLight.attach(&imAlive, 0.25);

 	isLogging = true;
 	rjLogLevel = OK;

	//initConsoleRoutine();

	// Create a new physical hardware communication link
 	CC1201 radio_900(
 		RJ_SPI_BUS,
 		RJ_PRIMARY_RADIO_CS,
 		RJ_PRIMARY_RADIO_INT
 		);

 	CC1201Config *radioConfig = new CC1201Config();
 	radioConfig = CC1201Config::resetConfiguration(radioConfig);
 	CC1201Config::loadConfiguration(radioConfig, &radio_900);
 	CC1201Config::verifyConfiguration(radioConfig, &radio_900);

 	radio_900.frequency();

	// Create a Communication Module Object
 	CommModule comm;
 	radio_900.setModule(comm);

 	comm.TxHandler((CommLink*)&radio_900, &CommLink::sendPacket, 8);
 	comm.RxHandler(rx_callback, 8);
 	comm.openSocket(8);

	// Create a dummy packet that is set to send out from socket connection 8
 	RTP_t dummy_packet;

	// Enable watchdog timer
	//Watchdog watchdog;
	//watchdog.set(RJ_WATCHDOG_TIMER_VALUE);

 	while(1) {

		dummy_packet.port = 8;
		dummy_packet.subclass = 1;
		dummy_packet.address = 255;
		dummy_packet.sfs = 0;
		dummy_packet.ack = 0;
		dummy_packet.field_size = 6;
		dummy_packet.payload_size = 10;

	 	for(int i=0; i<10; i++)
	        dummy_packet.payload[i] = 0xAA; // 0b10101010


	    uint8_t rssi_regs[2];
	    rssi_regs[0] = radio_900.readReg(0x72, EXT_FLAG_ON);
	    rssi_regs[1] = radio_900.readReg(0x71, EXT_FLAG_ON);
	    radio_900.rssi(rssi_regs);

	    log(OK, "MAIN LOOP", "  MARCSTATE: 0x%02X\r\n  RSSI: %.1f dBm", radio_900.mode(), radio_900.rssi());

	    radio_900.calibrate();
	    osDelay(100);

	    comm.send(dummy_packet);

	    for (int i=0; i<900; i++)
	    {
	    	osDelay(1);
	    	is_locked = radio_900.isLocked();
	    	rssi_valid = gpio2;
	    }
	}
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
	//interrupt will need to be deferred.
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
 			Thread::wait(100);
 			//ledTwo = !ledTwo;
 		}

		//clear light for main loop (shows its complete)
 		//ledTwo = false;
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
