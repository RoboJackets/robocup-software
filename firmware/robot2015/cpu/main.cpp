
#include "robot.hpp"
#include "console.hpp"
#include "commands.hpp"
#include "logger.hpp"
#include "radio.hpp"

#define READ_SPI_16	0x8000
#define SET_ADDR( x ) ((x)<<11)

LocalFileSystem local("local");


Ticker lifeLight;
DigitalOut ledOne(LED1);
DigitalOut ledTwo(LED2);
DigitalOut drv_ncs(p20,1);
DigitalOut drv_en(p19,1);

//  mosi, miso, sclk - connected to fpga
SPI spi(p5, p6, p7);

/*
 * some forward declarations to make the code easier to read
 */
void imAlive(void);
void setISRPriorities(void);
void initRadioThread(void);
void initConsoleRoutine(void);
void fpgaInit();

/**
 * system entry point
 */
int main(void) 
{
	//setISRPriorities();
	lifeLight.attach(&imAlive, 0.25);

	isLogging = true;
	rjLogLevel = INF2;
	
	//initRadioThread();
	//initConsoleRoutine();

    fpgaInit();

    spi.format(16,0);

    uint16_t reg_config [2];
    reg_config[0] = READ_SPI_16 | SET_ADDR(2) | (6<<6);
    reg_config[1] = READ_SPI_16 | SET_ADDR(3) | (1<<5) | (1<<4);
    
    for (int i=0; i<2; i++)
    	spi.write(reg_config[i]);

    uint16_t reg_vals[2];

    while(1)
    {

    	for (int i=0; i<2; i++)
    		reg_vals[i] = 0x3FF & spi.write(SET_ADDR(i));
/*
    	bool fault = reg_vals[0]>>10;
    	bool gvdd_uv = reg_vals[0]>>9;
    	bool pvdd_uv = reg_vals[0]>>8;
    	bool otsd = reg_vals[0]>>7;
    	bool otw = reg_vals[0]>>6;
    	bool ah_oc = reg_vals[0]>>5;
    	bool al_oc = reg_vals[0]>>4;
    	bool bh_oc = reg_vals[0]>>3;
    	bool bl_oc = reg_vals[0]>>2;
    	bool ah_oc = reg_vals[0]>>1;
    	bool al_oc = reg_vals[0];
    	*/

/*
    	printf("Fault:\t%u", fault);
    	printf("GVDD_UV\t%u", gvdd_uv);
    	printf("PVDD_UV\t%u", pvdd_uv);
    	printf("GVDD_UV\t%u", );
    	printf("GVDD_UV\t%u", gvdd_uv);
    	*/

    	printf("Address 0x00:\t0x%04X\r\nAddress 0x01:\t0x%04X\r\n", reg_vals[0], reg_vals[1]);
    	wait(2);
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

void fpgaInit() {
    DigitalOut fpgaCs(p18); //  chip select for SPI to fpga
    DigitalOut fpga_prog_b(p24);


    fpga_prog_b = 1;
    wait(1);    //  1 second
    fpga_prog_b = 0;
    wait_us(80);
    fpga_prog_b = 1;

    //  8 bits per write, mode 3?
    spi.format(8, 3);

    //  1MHz - pretty slow, we can boost this later
    //  our max physical limit is 1/8 of the fpga system clock (18.4MHz), so 18.4/8 is the max
    spi.frequency(1000000);

    FILE *fp = fopen("/local/robocup.nib", "r");

    printf("opened file: %p\r\n", fp);

    int result = 0;
    char buf[10];
    while (true) {
        size_t bytes_read = fread(buf, 1, 1, fp);
        if (bytes_read == 0) break;
        // printf("writing: %d...\r\n", buf[0]);
        result = spi.write(buf[0]); //  result should always be 0xFF since it's wired to high
    }

    fclose(fp);


    printf("got final byte from spi: %x\r\n\tFPGA configured!\r\n", result);
}

/**
 * timer interrupt based light flicker. If this stops, the code triggered
 * a fault.
 */
void imAlive()
{
	ledOne = !ledOne;
}
