#include "robot.hpp"
#include "radio.hpp"

/*
 * forward delarations
 */
void radioThreadHandler(void const* args);

/**
 * call to initialize radio
 */
int initRadio(void)
{
	Thread radioThread(radioThreadHandler);

	return 0;
}

void radioThreadHandler(void const* args)
{
	for (;;)
	{
		Thread::wait(500);
	}
}

/*
#include "robot.hpp"

#define PACKETS_PER_SEC 60
#define BULK_TRANSMITTER_EN 0

// Create a file system if needed for writing startup information to the boot log
#if RJ_BOOT_LOG
LocalFileSystem local("local");     // Create the local filesystem object
#endif
DigitalOut led1(LED1, 0);

Serial pc(USBTX, USBRX);

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

    DigitalOut temppp(RJ_PRIMARY_RADIO_INT, 1);
    temppp = 0;

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

// Start listening on the setup port number
    comm.openSocket(8);

// Create a dummy packet that is set to send out from socket connection 8
    RTP_t dummy_packet;

    dummy_packet.port = 8;
    dummy_packet.subclass = 1;

// Enable watchdog timer
    //Watchdog watchdog;
    //watchdog.set(RJ_WATCHDOG_TIMER_VALUE);
    
    led1 = 1;
    
    while (true) {

        led1 = !led1;
        
        DigitalOut tx_led(LED2, 0);

#if BULK_TRANSMITTER_EN == 1
        for(uint8_t i=0; i<PACKETS_PER_SEC; i++) {

            uint8_t sz = rand()%32+1;
            for (uint8_t j=0; j<sz; j++) {
                dummy_packet.data[j] = rand()%16;
            }

            dummy_packet.data_size = sz;

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
