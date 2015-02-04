#include "robot.hpp"
#include "console.hpp"
#include "commands.hpp"
#include "logger.hpp"
#include "radio.hpp"

Ticker lifeLight;
DigitalOut ledOne(LED1);
DigitalOut ledTwo(LED2);

/*
 * some forward declarations to make the code easier to read
 */
void imAlive(void);
void setISRPriorities(void);
void initRadioThread(void);
void initConsoleRoutine(void);

/**
 * system entry point
 */
int main(void) 
{
	setISRPriorities();
	lifeLight.attach(&imAlive, 0.25);

	isLogging = true;
	rjLogLevel = INF2;
	
	initRadioThread();
	initConsoleRoutine();
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


/*
#include "robot.h"

#define PACKETS_PER_SEC 60
#define RECEIVER 1

// Create a file system if needed for writing startup information to the boot log
#if RJ_BOOT_LOG
LocalFileSystem local("local");     // Create the local filesystem object
#endif
DigitalOut led1(LED1, 0);

Serial pc(USBTX, USBRX);


void rx_handler(RTP_t *p)
{
    std::printf("\r\n%u bytes received!\r\n", p->payload_size);
    std::printf("  RSSI: %d\r\n", p->rssi);
    std::printf("  LQI: %d\r\n", p->lqi & 0x7F);
}


// Sets the mbed's baudrate for debugging purposes
void baud(int baudrate)
{
    Serial s(USBTX, USBRX);
    s.baud(baudrate);
}


// Main program operations =======================
int main()
{
// Set the baud rate
    baud(57600);

// Check the mbed's firmware if enabled
#if RJ_CHECK_FIRMWARE
    std::string firmware;
    firmware_version(firmware);  // this is from FirmwareHelper.h
    LOG("Firmware Version: %s", firmware.c_str());

// Write any errors to a log file if enabled
#if RJ_BOOT_LOG
    LOG("Begin logging");
#endif

#endif

    //DigitalOut temppp(RJ_PRIMARY_RADIO_INT, 1);
    //temppp = 0;

// Create a new physical hardware communication link
    CC1101 radio_900(
        RJ_SPI_BUS,
        RJ_PRIMARY_RADIO_CS,
        RJ_PRIMARY_RADIO_INT
    );

// Create a Communication Module Object
    CommModule comm;

// Create a Hardware Link Object
    radio_900.setModule(comm);

// Set the callback funtion for sending a packet over a given port
    comm.TxHandler((CommLink*)&radio_900, &CommLink::sendPacket, 8);
    comm.RxHandler(rx_handler, 8);

// Start listening on the setup port number
    comm.openSocket(8);

// Create a dummy packet that is set to send out from socket connection 8
    RTP_t dummy_packet;

    dummy_packet.address = 0xFF;
    dummy_packet.sfs = 0;
    dummy_packet.ack = 0;
    dummy_packet.subclass = 1;
    dummy_packet.port = 8;

    for(int i=0; i<10; i++) {
        dummy_packet.payload[i] = 0xAA; // 0b10101010
    }
    dummy_packet.payload_size = 10;

#if RJ_WATCHDOG_EN
// Enable watchdog timer
    Watchdog watchdog;
    watchdog.set(RJ_WATCHDOG_TIMER_VALUE);
#endif

    led1 = 1;

    while(1) {

        led1 = !led1;
        DigitalOut tx_led(LED2, 0);

// Used for easier compiling when testing a transmitter and a receiver
#if RECEIVER == 0

        for(uint8_t i=0; i<PACKETS_PER_SEC; i++) {

            uint8_t sz = rand()%32+1;
            for (uint8_t j=0; j<sz; j++) {
                dummy_packet.payload[j] = rand()%16;
            }
            dummy_packet.payload_size = sz;

            comm.send(dummy_packet);
            tx_led = !tx_led;
            osDelay(15);
        }

#endif

        osDelay(500);
        tx_led = !tx_led;
        comm.send(dummy_packet);
        tx_led = 0;
        osDelay(500);
    }
}
*/