
#include "robot.hpp"
#include "console.hpp"
#include "commands.hpp"
#include "logger.hpp"
#include "radio.hpp"

#define READ_REG        (0x8000)
#define WRITE_REG       (0x0000)

#define GAIN_10         (0x00)
#define GAIN_20         (0x01)
#define GAIN_40         (0x02)
#define GAIN_80         (0x03)

#define SET_ADDR(x)     (((x)<<11) & 0x7800)
#define STATUS_REG(x)   SET_ADDR(x-1)
#define CTRL_REG(x)     SET_ADDR(+1)

#define GATE_CURRENT(x) ( (x)     & 0x003)
#define GATE_RESET(x)   (((x)<<2) & 0x004)
#define PWM_MODE(x)     (((x)<<3) & 0x008)
#define OC_MODE(x)      (((x)<<4) & 0x030)
#define OC_ADJ_SET(x)   (((x)<<6) & 0x7C0)

#define OCTW_SET(x)     ( (x)     & 0x003)
#define GAIN(x)         (((x)<<2) & 0x00C)
#define DC_CAL_CH1(x)   (((x)<<4) & 0x010)
#define DC_CAL_CH2(x)   (((x)<<5) & 0x020)
#define TC_OFF(x)       (((x)<<6) & 0x040)




Serial pc(USBTX, USBRX);
LocalFileSystem local("local");
SPI spi(p5, p6, p7);	//  mosi, miso, sclk - connected to fpga
Ticker lifeLight;

DigitalOut ledOne(LED1,1);
DigitalOut ledTwo(LED2,0);
DigitalOut drv_configured(LED3,0);

DigitalOut n_cs(p10,1);
DigitalOut gate_en(p15,0);


/*
 * some forward declarations to make the code easier to read
 */
void imAlive(void);
void setISRPriorities(void);
void initRadioThread(void);
void initConsoleRoutine(void);
void fpgaInit(void);
void DRV8303Init(void);

/**
 * system entry point
 */
int main(void)
{
	//setISRPriorities();
	lifeLight.attach(&imAlive, 0.25);

	isLogging = true;
	rjLogLevel = LOG_LEVEL::INFO;

	//initRadioThread();
	//initConsoleRoutine();

    fpgaInit();
    DRV8303Init();

	uint16_t data[3];
	uint32_t transfer_num = 0;

    while(1) {
        for(uint8_t i=0; i<5; i++) {
            n_cs = 0;
            data[i] = spi.write(SET_ADDR(i) | READ_REG);
            n_cs = ~n_cs;
        }

        printf("\r\nReading %u:\r\n", ++transfer_num);
        for(uint8_t i=0; i<4; i++)
            printf("    Address %u:\t%04X\r\n", i, data[i+1]);

        wait(5);
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
		Console::Init();

		while (true)
		{
			//check console communications, currently does nothing
			//then execute any active iterative command
			Console::ConComCheck();
			//execute any active iterative command
			executeIterativeCommand();

			//check if a system stop is requested
			if (Console::IsSystemStopRequested() == true)
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
		while (true);
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

void DRV8303Init()
{
    pc.baud(57600);
    spi.format(16,1);
    spi.frequency(1000000);
    gate_en = 0;
    wait(0.5);
    gate_en = !gate_en;

    uint16_t data[3];
    data[0] = CTRL_REG(1) | OC_ADJ_SET(10) | WRITE_REG;
    data[1] = CTRL_REG(2) | GAIN(GAIN_20) | WRITE_REG;

    printf("Configuration Values:\t%04X\t%04X\r\n", data[0], data[1]);
    for(uint8_t i=0; i<3; i++) {
        n_cs = 0;
        data[2] = spi.write(data[i]);
        n_cs = ~n_cs;
    }
    drv_configured = 1;
}


/**
 * timer interrupt based light flicker. If this stops, the code triggered
 * a fault.
 */
void imAlive()
{
	ledOne = !ledOne;
}
