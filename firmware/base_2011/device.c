#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>

#include "bits.h"
#include "usb.h"
#include "radio.h"
#include "cc1101.h"

typedef struct {
    uint8_t len;
    uint8_t data[64];
} buffer_t;

#define RX_QUEUE_SIZE 8
buffer_t rx_queue[RX_QUEUE_SIZE];
uint8_t rx_queue_write, rx_queue_read;

// Nonzero when we are transmitting.
// Used by GDO2 interrupt.
volatile uint8_t in_tx = 0;

// Used for putting words in descriptor byte arrays
#define WORD(x) ((x)&0xff), (((x) >> 8) & 0xff)

const uint8_t device_desc[] PROGMEM = {
    18,            // bLength
    1,             // bDescriptorType
    WORD(0x200),   // bcdUSB
    0xff,          // bDeviceClass
    0,             // bDeviceSubclass
    0,             // bDeviceProtocol
    32,            // bMaxPacketSize0
    WORD(0x3141),  // idVendor
    WORD(0x0004),  // idProduct
    WORD(0),       // bcdDevice
    0,             // iManufacturer
    0,             // iProduct
    0,             // iSerialNumber
    1              // bNumConfigurations
};

#define CONFIG_SIZE 9
#define INTF_SIZE 9
#define EP_SIZE 7
const uint8_t config_desc[] PROGMEM = {
    // Configuration
    CONFIG_SIZE,  // bLength
    2,            // bDescriptorType
    WORD(         // wTotalLength
        CONFIG_SIZE + INTF_SIZE + EP_SIZE * 2),
    1,     // bNumInterfaces
    1,     // bConfigurationValue
    0,     // iConfiguration
    0xc0,  // bmAttributes
    0,     // MaxPower

    // Interface 0
    INTF_SIZE,  // bLength
    4,          // bDescriptorType
    0,          // bInterfaceNumber
    0,          // bAlternateSetting
    2,          // bNumEndpoints (not counting endpoint 0)
    0xff,       // bInterfaceClass
    0xff,       // bInterfaceSubclass
    0,          // bInterfaceProtocol
    0,          // iInterface

    // Endpoint 1: bulk OUT
    EP_SIZE,   // bLength
    5,         // bDescriptorType
    1,         // bEndpointAddress
    2,         // bmAttributes
    WORD(64),  // wMaxPacketSize
    0,         // bInterval

    // Endpoint 2: bulk IN
    EP_SIZE,   // bLength
    5,         // bDescriptorType
    0x82,      // bEndpointAddress
    2,         // bmAttributes
    WORD(64),  // wMaxPacketSize
    0          // bInterval
};

// Read n bytes from the current endpoint
void usb_read(void* buf, uint8_t n) {
    while (n--) {
        *(uint8_t*)buf++ = UEDATX;
    }
}

// Sends the data currently in the FIFO (typically used to send a zero-length
// packet)
void usb_write_packet() {
    clear_bit(UEINTX, TXINI);
    loop_until_bit_is_set(UEINTX, TXINI);
}

// Send data from program memory
void usb_write_P(PGM_VOID_P buf, uint8_t len, uint16_t requested) {
    // Only send up to the requested amount of data.
    // Otherwise we will be waiting forever for another IN token from the host.
    uint8_t n = len;
    if (n > requested) {
        n = requested;
    }

    uint8_t sent = 0;
    while (n--) {
        UEDATX = pgm_read_byte(buf++);
        ++sent;

        if (n == 0 || sent == 64)  // FIXME - Endpoint size
        {
            sent = 0;
            clear_bit(UEINTX, TXINI);
            while (!(UEINTX & ((1 << TXINI) | (1 << RXOUTI))))
                ;
            if (bit_is_set(UEINTX, RXOUTI)) {
                // Aborted by host
                return;
            }
        }
    }

    // Wait for status
    loop_until_bit_is_set(UEINTX, RXOUTI);
    clear_bit(UEINTX, RXOUTI);
}

// Send data from RAM
void usb_write(void* buf, uint8_t len, uint16_t requested) {
    // Only send up to the requested amount of data.
    // Otherwise we will be waiting forever for another IN token from the host.
    uint8_t n = len;
    if (n > requested) {
        n = requested;
    }

    uint8_t sent = 0;
    while (n--) {
        UEDATX = *(uint8_t*)buf++;
        ++sent;

        if (n == 0 || sent == 64)  // FIXME - Endpoint size
        {
            sent = 0;
            clear_bit(UEINTX, TXINI);
            while (!(UEINTX & ((1 << TXINI) | (1 << RXOUTI))))
                ;
            if (bit_is_set(UEINTX, RXOUTI)) {
                // Aborted by host
                return;
            }
        }
    }

    // Wait for status
    loop_until_bit_is_set(UEINTX, RXOUTI);
    clear_bit(UEINTX, RXOUTI);
}

// Starts the bootloader
void bootloader() {
    cli();
    set_bit(UDCON, DETACH);
    ((void (*)(void))0x1800)();
}

// Disables and frees memory for all endpoints except 0.
void usb_free_endpoints() {
    uint8_t old_ep = UENUM;

    uint8_t i;
    for (i = 1; i <= 4; ++i) {
        UENUM = i;
        clear_bit(UECONX, EPEN);
        clear_bit(UECFG1X, ALLOC);
    }

    UENUM = old_ep;
}

// Called when a SETUP packet is received on endpoint 0.
void usb_handle_setup() {
    // Read the packet
    USB_Setup_Request req;
    usb_read(&req, 8);

    // clear_bit the FIFO
    // FIXME - Do this later if this is an OUT transfer (host will be sending in
    // the DATA stage)?
    clear_bit(UEINTX, RXSTPI);

    // Every request must end with either an IN packet (usb_write_packet or
    // similar) or
    // a STALL, which is sent if valid == 0.
    uint8_t valid = 1;

    if ((req.bmRequestType & 0x60) == 0) {
        // Device request
        switch (req.bRequest) {
            case 0x01:  // clear_bit feature
                if (req.wValue == 0 && req.bmRequestType == 0x02) {
                    // Endpoint halt
                    UENUM = req.wIndex;
                    set_bit(UECONX, RSTDT);
                    UENUM = 0;
                } else {
                    valid = 0;
                    break;
                }

                usb_write_packet();
                break;

            // FIXME - Set feature

            case 0x05:  // Set address
                UDADDR = (UDADDR & (1 << ADDEN)) | (req.wValue & 0x7f);
                usb_write_packet();
                set_bit(UDADDR, ADDEN);
                break;

            case 0x06:  // Get descriptor
                if (req.wValue == 0x0100) {
                    // Get device descriptor
                    usb_write_P(device_desc, sizeof(device_desc), req.wLength);
                } else if (req.wValue == 0x0200) {
                    // Get configuration descriptor
                    usb_write_P(config_desc, sizeof(config_desc), req.wLength);
                } else {
                    valid = 0;
                }
                break;

            case 0x09:  // Set configuration
                usb_free_endpoints();

                if (req.wValue == 1) {
                    // Configuration 1
                    // Endpoint 3: bulk OUT
                    //   64-byte packets
                    //   Single buffer
                    UENUM = 1;
                    UECONX = (1 << EPEN);  // Do this before other configuration
                    UEIENX = (1 << RXOUTE);
                    UECFG0X = (2 << EPTYPE0);
                    UECFG1X = (3 << EPSIZE0) | (1 << EPBK0);
                    set_bit(UECFG1X, ALLOC);

                    // Endpoint 2: bulk IN
                    //   64-byte packets
                    //   Single buffer
                    UENUM = 2;
                    UECONX = (1 << EPEN);  // Do this before other configuration
                    UEIENX = 0;
                    UECFG0X = (2 << EPTYPE0) | (1 << EPDIR);
                    UECFG1X = (3 << EPSIZE0) | (1 << EPBK0);
                    set_bit(UECFG1X, ALLOC);

                    set_bit(PORTB, 4);
                    UENUM = 0;
                } else if (req.wValue == 0) {
                    // Configuration 0 (unconfigured)
                    clear_bit(PORTB, 4);
                } else {
                    valid = 0;
                    break;
                }

                // Clear the packet queue
                rx_queue_read = 0;
                rx_queue_write = 0;
                for (uint8_t i = 0; i < RX_QUEUE_SIZE; ++i) {
                    rx_queue[i].len = 0;
                }
                clear_bit(PORTB, 7);

                usb_write_packet();
                break;

            default:
                valid = 0;
                break;
        }
    } else if ((req.bmRequestType & 0x7f) == 0x40) {
        // Vendor request
        switch (req.bRequest) {
            case 0:  // LEDs
                PORTB = (PORTB & 0x0f) | ((req.wValue & 0x0f) << 4);
                usb_write_packet();
                break;

            case 1:  // Write register
                radio_write(req.wIndex, req.wValue);
                usb_write_packet();
                break;

            case 2:  // Command
                radio_command(req.wIndex);
                usb_write_packet();
                break;

            case 3:  // Read register
                if ((req.bmRequestType & 0x80) && req.wLength > 0) {
                    uint8_t value = radio_read(req.wIndex);
                    usb_write(&value, 1, req.wLength);
                } else {
                    valid = 0;
                }
                break;

            case 4:  // Mode
                if (req.wValue) {
                    radio_timeout_enable();
                } else {
                    radio_timeout_disable();
                }
                usb_write_packet();
                break;

            case 0xff:  // Bootloader
                usb_write_packet();
                bootloader();
                break;

            default:
                valid = 0;
                break;
        }
    } else {
        valid = 0;
    }

    if (!valid) {
        // Unsupported request: stall EP0
        set_bit(UECONX, STALLRQ);
    }
}

void handle_ep1_rx() {
    // Received TX data on endpoint 1.
    set_bit(PORTB, 6);

    // Turn off the RX calibration timer.
    // It will be reset and turned back on when TX is done.
    radio_timeout_disable();

    // Go to IDLE.  This only takes 2 radio clocks.
    radio_command(SIDLE);

    // Flush the RX FIFO in case we were receiving a packet
    // and cut it off.
    radio_read(RXFIFO);
    radio_command(SFRX);

    // Flush the TX FIFO
    radio_command(SFTX);

    // Copy data to the radio transmit FIFO
    radio_select();
    spi_write(TXFIFO | CC_BURST);
    spi_write(UEBCLX);
    while (UEBCLX) {
        spi_write(UEDATX);
    }

    // Acknowledge the interrupt
    clear_bit(UEINTX, RXOUTI);

    radio_deselect();

    in_tx = 1;

    // Turn on autocalibration so we calibrate before sending this packet
    radio_write(MCSM0, radio_read(MCSM0) | 0x10);

    // Start transmission
    radio_command(STX);

    // Turn off autocalibration so we don't calibrate when switching to RX
    // after the packet is sent
    radio_write(MCSM0, radio_read(MCSM0) & 0x0f);
}

void handle_radio_tx() {
    // The radio finished transmitting a packet.
    // The state transition for TXOFF_MODE has started.

    // Go to RX
    //     radio_command(SFRX);
    radio_command(SRX);

    // Reset and enable the RX timeout timer
    radio_timeout_enable();

    // The next interrupt will be for received data.
    in_tx = 0;

    // Free the bank
    UENUM = 1;
    clear_bit(UEINTX, FIFOCON);

    // Turn off the TX LED
    clear_bit(PORTB, 6);
}

void handle_radio_rx() {
    // The radio received a packet.

    // Reset the calibration timer
    radio_timeout_enable();

    uint8_t bytes = radio_read(RXBYTES);
    if (bytes < 4) {
        // No data means bad CRC, so the packet was flushed automatically.
        radio_command(SRX);
        return;
    }

    // Copy FREQEST to FREQOFF
    radio_write(FSCTRL0, radio_read(FREQEST));

    // Get the next available RX buffer
    buffer_t* buf = &rx_queue[rx_queue_write];

    if (buf->len) {
        UENUM = 2;
        UEIENX = (1 << TXINE);
        // All buffers are full, so discard this packet
        set_bit(PORTB, 5);
        radio_deselect();
        radio_command(SFRX);
        radio_command(SRX);
        return;
    }

    // Read RXFIFO (burst)
    radio_select();
    spi_write(RXFIFO | CC_READ | CC_BURST);
    uint8_t pktlen = spi_write(SNOP);

    if (bytes != (pktlen + 3)) {
        // Wrong number of bytes in FIFO - probably dropped data
        radio_deselect();
        radio_command(SFRX);
        radio_command(SRX);
        return;
    }

    // Don't send the packet length to the host, since it will be implied in the
    // USB transfer size
    --bytes;

    rx_queue_write += 1;
    if (rx_queue_write == RX_QUEUE_SIZE) {
        rx_queue_write = 0;
    }

    set_bit(PORTB, 7);
    buf->len = bytes;
    for (uint8_t i = 0; i < bytes; ++i) {
        buf->data[i] = spi_write(SNOP);
        // We don't care about multiple packets because the FIFO size on the
        // radio is the maximum USB packet size.
    }
    radio_deselect();

    // Enable the transmit interrupt
    UENUM = 2;
    UEIENX = (1 << TXINE);

    //     clear_bit(PORTB, 7);

    radio_command(SRX);
}

void handle_ep2_tx() {
    // Ready to transmit data on IN endpoint 2
    clear_bit(PORTB, 7);
    clear_bit(PORTB, 5);

    buffer_t* buf = &rx_queue[rx_queue_read];
    rx_queue_read += 1;
    if (rx_queue_read == RX_QUEUE_SIZE) {
        rx_queue_read = 0;
    }

    // Copy to the USB endpoint
    for (uint8_t i = 0; i < buf->len; ++i) {
        UEDATX = buf->data[i];
    }

    // Mark this buffer as empty
    buf->len = 0;

    // Mark the endpoint FIFO as ready to be sent
    clear_bit(UEINTX, TXINI);
    clear_bit(UEINTX, FIFOCON);

    if (rx_queue[rx_queue_read].len == 0) {
        // Next buffer is empty - nothing to send
        UEIENX = 0;
    }
}

void device_gdo0_vect() {}

void device_gdo2_vect() {
    if (in_tx) {
        handle_radio_tx();
    } else {
        handle_radio_rx();
    }
}

void device_gen_vect() {
    if (bit_is_set(UDINT, WAKEUPI)) {
        // Wakeup (USB bus activity is detected)

        // We have to turn off the interrupt or it will trigger frequently when
        // the device is active.  This causes all sorts of weird problems.
        clear_bit(UDIEN, WAKEUPE);

        // Start the 48MHz PLL
        set_bit(PLLCSR, PLLE);
        loop_until_bit_is_set(PLLCSR, PLOCK);

        // Enable the USB clock
        clear_bit(USBCON, FRZCLK);

        // Clear the interrupt flag after starting the clock
        clear_bit(UDINT, WAKEUPI);

        // The USB controller will indicate end-of-reset soon.
        // The end-of-reset handler will set up endpoints.
    }

    if (bit_is_set(UDINT, EORSTI)) {
        // End of USB reset
        clear_bit(UDINT, EORSTI);

        usb_free_endpoints();

        // Turn off the green LED
        clear_bit(PORTB, 4);

        // Endpoint 0: control
        //   32-byte packets (so we can send the device and config descriptors
        //   in one packet)
        //   Single buffer
        UENUM = 0;
        clear_bit(UECONX, EPEN);
        clear_bit(UECFG1X, ALLOC);
        UECONX = (1 << EPEN);  // Do this before other configuration
        UEIENX = (1 << RXSTPE);
        UECFG0X = 0;
        UECFG1X = (2 << EPSIZE0);
        set_bit(UECFG1X, ALLOC);

        // Other endpoints will be configured in response to the
        // set_configuration request
    }

    if (bit_is_set(UDINT, SUSPI)) {
        // Suspend (bus idle)

        // Turn off the green LED
        clear_bit(PORTB, 4);
    }
}

void device_com_vect() {
    UENUM = 0;
    if (bit_is_set(UEINTX, RXSTPI)) {
        usb_handle_setup();
    }

    UENUM = 1;
    if (bit_is_set(UEINTX, RXOUTI)) {
        handle_ep1_rx();
    }

    UENUM = 2;
    if (bit_is_set(UEIENX, TXINE) && bit_is_set(UEINTX, TXINI)) {
        handle_ep2_tx();
    }
}

void device_main() {
    cli();

    rx_queue_read = 0;
    rx_queue_write = 0;
    for (uint8_t i = 0; i < RX_QUEUE_SIZE; ++i) {
        rx_queue[i].len = 0;
    }

    // Device mode
    set_bit(UHWCON, UIMOD);

    // Reattach
    set_bit(UDCON, DETACH);
    clear_bit(UDCON, DETACH);

    // Enable interrupts
    UDIEN = (1 << WAKEUPE) | (1 << EORSTE) | (1 << SUSPE);

    // Configure external interrupt 6 (GDO2 from radio)
    radio_write(IOCFG2, 6 | GDOx_INVERT);
    set_bit(EIFR, 2);

    sei();

    sleep_enable();
    while (1) {
        sleep_cpu();
    }
}
