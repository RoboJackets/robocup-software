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
    std::printf("%u bytes received.\r\n", p->payload_size);
    std::printf("RSSI: %d\r\n", p->rssi);
    std::printf("LQI: %d\r\n", p->lqi & 0x7F);
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

    dummy_packet.address = 0x1A;
    dummy_packet.sfs = 0;
    dummy_packet.ack = 0;
    dummy_packet.subclass = 1;
    dummy_packet.port = 8;

    for(int i=0; i<10; i++) {
        dummy_packet.payload[i] = 0xFF;
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

#if RECEIVER == 0
        //unsigned int wait_time = (1/PACKETS_PER_SEC)*1000;

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
