#include <stdio.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>

#include "bits.h"
#include "usb.h"
#include "radio.h"
#include "cc1101.h"

#define RED1 7
#define GREEN1 6
#define GREEN2 5
#define RED2 4

#define printf_P(x, ...)

enum {
    // No device is connected.
    USB_Disconnected,

    // A device is connected but not configured.
    USB_Connected,

    // A device is connected, configured, and ready to use.
    USB_Configured,

    // An error occurred during connection or configuration.
    // The device can't be used until it is reconnected.
    USB_Broken
};

volatile uint8_t usb_state = USB_Disconnected;

enum { Dev_Unknown, Dev_WiredGamepad, Dev_RumblePad2 };

uint8_t device_type = Dev_Unknown;

// Joystick inputs
uint8_t left_x, left_y, right_x, right_y, buttons[2];

// Dribbling is stateful.  Releasing button 5 leaves the dribbler in its last
// state.
uint8_t dribble;

#define BUTTON_1 (buttons[0] & 0x10)
#define BUTTON_2 (buttons[0] & 0x20)
#define BUTTON_3 (buttons[0] & 0x40)
#define BUTTON_4 (buttons[0] & 0x80)
#define BUTTON_5 (buttons[1] & 0x01)
#define BUTTON_6 (buttons[1] & 0x02)
#define BUTTON_7 (buttons[1] & 0x04)
#define BUTTON_8 (buttons[1] & 0x08)
#define BUTTON_9 (buttons[1] & 0x10)
#define BUTTON_10 (buttons[1] & 0x20)

uint8_t selected_robot = 0;
uint8_t forward_packet[Forward_Size];

void usb_send() {
    // Clear the interrupt flag.
    clear_bit(UPINTX, TXOUTI);

    // Start the transfer.
    clear_bit(UPINTX, FIFOCON);
}

void usb_wait(int ms) {
    clear_bit(UHINT, HSOFI);
    while (ms--) {
        loop_until_bit_is_set(UHINT, HSOFI);
        clear_bit(UHINT, HSOFI);
    }
}

static inline int usb_cur_pipe_size() {
    return 1 << (((UPCFG1X & 0x30) >> 4) + 3);
}

// Sets the packet size of the current pipe.
// s must be a power of two on the range [8, 256].
void usb_set_cur_pipe_size(int s) {
    char i;
    for (i = 3; i <= 8; ++i) {
        if (s & (1 << i)) {
            break;
        }
    }

    UPCFG1X &= ~0x30;
    UPCFG1X |= (i - 3) << 4;
}

// Waits for an IN packet or a STALL.
// Returns nonzero on IN or zero on STALL.
char usb_wait_rx() {
    while (1) {
        if (bit_is_set(UPINTX, RXINI)) {
            clear_bit(UPINTX, RXINI);
            return 1;
        }

        if (bit_is_set(UPINTX, RXSTALLI)) {
            clear_bit(UPINTX, RXSTALLI);
            return 0;
        }
    }
}

void usb_setup_write(const USB_Setup_Request* c) {
    usb_wait(1);

    UPNUM = 0;
    UPCFG0X = 0x00;

    uint8_t i;
    const uint8_t* data = (const uint8_t*)c;
    for (i = 0; i < 8; ++i) {
        UPDATX = *(uint8_t*)data++;
    }

    clear_bit(UPINTX, TXSTPI);
    clear_bit(UPCONX, PFREEZE);
    usb_send();
    loop_until_bit_is_set(UPINTX, TXSTPI);
    clear_bit(UPINTX, TXSTPI);
    set_bit(UPCONX, PFREEZE);
}

uint16_t usb_control_in(const USB_Setup_Request* c, uint8_t* data) {
    usb_setup_write(c);

    uint16_t read_len = 0;
    if (data) {
        // Data stage (IN)
        UPCFG0X = 0x10;

        while (read_len < c->wLength) {
            // Wait for IN data or a STALL
            clear_bit(UPCONX, PFREEZE);
            char ret = usb_wait_rx();
            set_bit(UPCONX, PFREEZE);

            if (ret == 0) {
                // Stalled
                return 0;
            }

            // Read this packet
            char partial = (UPBCX < usb_cur_pipe_size());
            while (UPBCX && read_len < c->wLength) {
                uint8_t byte = UPDATX;
                *data++ = byte;
                ++read_len;
            }

            clear_bit(UPINTX, FIFOCON);

            if (partial) {
                break;
            }
        }
    }

    // Status stage (OUT)
    UPCFG0X = 0x20;

    clear_bit(UPCONX, PFREEZE);
    loop_until_bit_is_set(UPINTX, TXOUTI);
    usb_send();
    loop_until_bit_is_set(UPINTX, TXOUTI);
    set_bit(UPCONX, PFREEZE);

    return read_len;
}

char usb_control_out(const USB_Setup_Request* c, uint8_t* data) {
    usb_setup_write(c);

#if 0
    // Data stage (OUT)
    UPCFG0X = 0x20;

    //FIXME - Send data
    clear_bit(UPCONX, PFREEZE);
//    loop_until_bit_is_set(UPINTX, TXOUTI);
//    usb_send();
//    loop_until_bit_is_set(UPINTX, TXOUTI);
    set_bit(UPCONX, PFREEZE);
#endif

    // Status stage (IN).
    // The device will send either a STALL or a zero-length IN packet.
    UPCFG0X = 0x10;

    // Wait for a response.
    clear_bit(UPCONX, PFREEZE);
    char ret = usb_wait_rx();
    set_bit(UPCONX, PFREEZE);

    // Clear interrupt flags.
    clear_bit(UPINTX, RXINI);
    clear_bit(UPINTX, RXSTALLI);

    if (ret) {
        // Clear FIFOCON to mark the buffer as free.
        // Only do this if RXIN was set (not stalled).
        clear_bit(UPINTX, FIFOCON);
    }

    return ret;
}

char identify_device(uint16_t vid, uint16_t pid) {
    if (vid == 0x046d) {
        printf_P(PSTR("Logitech "));

        if (pid == 0xc216) {
            printf_P(PSTR("Dual Action Gamepad"));
            return Dev_WiredGamepad;
        } else if (pid == 0xc219) {
            printf_P(PSTR("Cordless RumblePad 2"));
            return Dev_RumblePad2;
        }
    }

    printf_P(PSTR("(unknown)"));

    return Dev_Unknown;
}

// Reads descriptors and sets up the device.
// Called immediately after the device is connected and reset.
char configure_device() {
    // Read the first 8 bytes of the device descriptor.
    // The last byte is the endpoint size on the device.
    // We can only assume that it is at least 8, so we have to
    // read the full descriptor again when we know the true value.
    USB_Setup_Request c;
    c.bmRequestType = 0x80;
    c.bRequest = 6;
    c.wValue = 0x0100;
    c.wIndex = 0;
    c.wLength = 8;
    uint8_t desc[18];
    if (usb_control_in(&c, desc) != 8) {
        // Failed to read device descriptor
        return 0;
    }

    if (desc[0] != 0x12 || desc[1] != 0x01) {
        // Bad device descriptor
        return 0;
    }

    // Update endpoint size and reallocate its memory.
    clear_bit(UPCONX, PEN);          // Disable and reset
    set_bit(UPCONX, PEN);            // Re-enable
    set_bit(UPCONX, INMODE);         // Allow unlimited IN tokens
    usb_set_cur_pipe_size(desc[7]);  // Set packet size
    set_bit(UPCONX, ALLOC);          // Allocate buffer memory

    // Read the full device descriptor
    c.wLength = 18;
    if (usb_control_in(&c, desc) != 18) {
        // Failed to read device descriptor
        return 0;
    }

    uint16_t vid = desc[8] | (desc[9] << 8);
    uint16_t pid = desc[10] | (desc[11] << 8);
    printf_P(PSTR("  ID %04x:%04x: "), vid, pid);
    device_type = identify_device(vid, pid);
    printf_P(PSTR("\n"));

    // Set address
    c.bmRequestType = 0x00;
    c.bRequest = 5;
    c.wValue = 1;
    c.wIndex = 0;
    c.wLength = 0;
    if (!usb_control_out(&c, 0)) {
        // Failed to set address
        return 0;
    }
    UHADDR = 1;

    // Set configuration 1
    c.bmRequestType = 0x00;
    c.bRequest = 9;
    c.wValue = 1;
    c.wIndex = 0;
    c.wLength = 0;
    if (!usb_control_out(&c, 0)) {
        // Failed to set configuration
        return 0;
    }

    // Set up pipe 1 for EP1 interrupt IN
    UPNUM = 1;
    set_bit(UPCONX, PEN);
    set_bit(UPCONX, INMODE);
    UPCFG0X = (3 << PTYPE0) | (1 << PTOKEN0) | 1;
    UPCFG1X = (1 << ALLOC);
    UPCFG2X = 10;
    clear_bit(UPCONX, PFREEZE);
    UPIENX = (1 << RXINE);

    return 1;
}

void host_gen_vect() {
    if (bit_is_set(OTGINT, SRPI)) {
        // Device detected while VBus is forced on.
        clear_bit(OTGINT, SRPI);

        printf_P(PSTR("Device connected\n"));

        // Switch to the Host Ready state.
        // We have to do this even though we're manually controlling VBus
        // because
        // the USB core needs to change state.
        set_bit(OTGCON, VBUSREQ);

        // DCONNI will be set shortly.
    }

    if (bit_is_set(UHINT, DCONNI)) {
        // Device connected.
        clear_bit(UHINT, DCONNI);

        // Reset the bus.  After the reset, SOFEN will be set automatically.
        // The bus activity will keep VBus on until the device is disconnected.
        set_bit(UHCON, RESET);

        // Configuration happens in response to RSTI.
    }

    if (bit_is_set(UHINT, DDISCI)) {
        // Device disconnected.
        clear_bit(UHINT, DDISCI);

        usb_state = USB_Disconnected;
        printf_P(PSTR("Device disconnected\n"));

        // Turn off all pipes to reset them.
        for (char i = 0; i < 6; ++i) {
            UPNUM = i;
            clear_bit(UPCONX, PEN);
        }

        // Move the USB core to the Host Idle state.
        // We have to do this even though we're manually controlling VBus
        // because
        // the USB core needs to change state.
        set_bit(OTGCON, VBUSRQC);

        // Turn off Vbus long enough for the USB core to notice, then turn it
        // back on.
        // This is required for the next device connection to be detected.
        // FIXME - Sometimes it still doesn't work.
        //      Keeping vbus off longer doesn't help.  Clearing SOFEN doesn't
        //      help.
        clear_bit(USBINT, VBUSTI);
        clear_bit(PORTE, 7);
        loop_until_bit_is_set(USBINT, VBUSTI);
        set_bit(PORTE, 7);
    }

    if (bit_is_set(UHINT, RSTI)) {
        // Reset finished.
        clear_bit(UHINT, RSTI);

        // Configure pipe 0 for EP0 control
        UPNUM = 0;
        UPCONX = (1 << PFREEZE) | (1 << PEN);
        UPCFG1X = (3 << PSIZE0) | (1 << ALLOC);

        usb_state = USB_Connected;
    }
}

void host_com_vect() {
    UPNUM = 1;
    if (usb_state == USB_Configured && bit_is_set(UPINTX, RXINI)) {
        clear_bit(UPINTX, RXINI);

        uint8_t len = UPBCLX;
        uint8_t i, buf[8];
        for (i = 0; i < len; ++i) {
            buf[i] = UPDATX;
        }
        clear_bit(UPINTX, FIFOCON);

        switch (device_type) {
            case Dev_WiredGamepad:
                left_x = buf[0];
                left_y = buf[1];
                right_x = buf[2];
                right_y = buf[3];
                buttons[0] = buf[4];
                buttons[1] = buf[5];
                break;

            case Dev_RumblePad2:
                left_x = buf[1];
                left_y = buf[2];
                right_x = buf[3];
                right_y = buf[4];
                buttons[0] = buf[5];
                buttons[1] = buf[6];
                break;

            default:
                left_x = left_y = 0;
                right_x = right_y = 0;
                buttons[0] = buttons[1] = 0;
                dribble = 0;
                break;
        }

#if 0
        if (ser_tx_empty())
        {
#if 1
            printf("%02x:  ", len);
            for (i = 0; i < len; ++i)
            {
                printf_P(PSTR("%02x "), buf[i]);
            }
#endif
            printf_P(PSTR("  %4d %4d %4d"), drive_x, drive_y, drive_spin);
            printf_P(PSTR("\n"));
        }
#endif
    }
}

ISR(TIMER0_COMPA_vect) {
    static uint8_t div = 0;

    // Divider for LED flashing
    ++div;
    div &= 15;

    if (usb_state == USB_Disconnected && div == 0) {
        PORTB ^= 0x10;
    } else if (usb_state == USB_Configured) {
        PORTB |= 0x10;
    } else if ((usb_state == USB_Connected || usb_state == USB_Broken) &&
               div == 0) {
        PORTB |= 0x20;
        PORTB ^= 0x10;
    }

    set_bit(PORTB, 6);

    if (usb_state != USB_Configured) {
        // No usable device connected: reset joystick inputs
        left_x = left_y = 0x80;
        right_x = right_y = 0x80;
        buttons[0] = buttons[1] = 0;
        dribble = 0;
    }

    // Driving
    int8_t drive_x = right_x - 0x80;
    int8_t drive_y = 0x7f - right_y;
    int8_t drive_spin = 0x7f - left_x;

    // Kick/chip
    uint8_t kick_strength;
    uint8_t chip_select;
    if (BUTTON_8) {
        chip_select = 0;
        kick_strength = 0xff;
    } else if (BUTTON_6) {
        chip_select = 1;
        kick_strength = 0xff;
    } else {
        chip_select = 0;
        kick_strength = 0;
    }

    // Dribble
    if (BUTTON_5) {
        if (left_y <= 0x7f) {
            dribble = (0x7f - left_y) >> 3;
        } else {
            dribble = 0;
        }
    }

    // Robot selection
    if (BUTTON_10) {
        selected_robot = 0;
    } else if (BUTTON_1) {
        selected_robot = 1;
    } else if (BUTTON_2) {
        selected_robot = 2;
    } else if (BUTTON_3) {
        selected_robot = 3;
    } else if (BUTTON_4) {
        selected_robot = 4;
    }

    // Sequence number and reverse board ID (no reverse packet, since we don't
    // have any use for it)
    forward_packet[0] = ((forward_packet[0] & 0xf0) + 0x10) | 0x0f;

    uint8_t offset = 1;
    // Leave the first four slots empty
    for (uint8_t i = 0; i < 4; ++i) {
        forward_packet[offset++] = 0;
        forward_packet[offset++] = 0;
        forward_packet[offset++] = 0;
        forward_packet[offset++] = 0;
        forward_packet[offset++] = 0x0f;
        forward_packet[offset++] = 0;
    }

    // Wheel layout:
    // 1  2
    // 0  3
    int16_t w0 = -drive_spin - drive_x + drive_y;
    int16_t w1 = -drive_spin + drive_x + drive_y;
    int16_t w2 = -drive_spin + drive_x - drive_y;
    int16_t w3 = -drive_spin - drive_x - drive_y;

    if (w0 > 127) {
        w0 = 127;
    } else if (w0 < -127) {
        w0 = -127;
    }

    if (w1 > 127) {
        w1 = 127;
    } else if (w1 < -127) {
        w1 = -127;
    }

    if (w2 > 127) {
        w2 = 127;
    } else if (w2 < -127) {
        w2 = -127;
    }

    if (w3 > 127) {
        w3 = 127;
    } else if (w3 < -127) {
        w3 = -127;
    }

    // The last slot is for the robot we're controlling
    forward_packet[offset++] = w0;
    forward_packet[offset++] = w1;
    forward_packet[offset++] = w2;
    forward_packet[offset++] = w3;
    forward_packet[offset++] = (dribble << 4) | selected_robot;
    forward_packet[offset++] = kick_strength;

    radio_command(SIDLE);
    radio_command(SFTX);

    radio_select();
    spi_write(TXFIFO | CC_BURST);
    spi_write(Forward_Size);
    for (uint8_t i = 0; i < Forward_Size; ++i) {
        spi_write(forward_packet[i]);
    }
    radio_deselect();

    // Start transmission
    radio_command(STX);
}

void host_gdo0_vect() {}

void host_gdo2_vect() { clear_bit(PORTB, 6); }

const uint8_t cc1101_regs[] = {
    0x0b, 0x0c,  // FSCTRL1  - Frequency synthesizer control.
    //    0x0c, 0x00,    // FSCTRL0  - Frequency synthesizer control.
    0x0d, 0x21,  // FREQ2    - Frequency control word, high byte.
    0x0e, 0x7b,  // FREQ1    - Frequency control word, middle byte.
    0x0f, 0x42,  // FREQ0    - Frequency control word, low byte.
    0x10, 0x2d,  // MDMCFG4  - Modem configuration.
    0x11, 0x2f,  // MDMCFG3  - Modem configuration.
    0x12, 0x13,  // MDMCFG2  - Modem configuration.
    0x13, 0x22,  // MDMCFG1  - Modem configuration.
    0x14, 0xe5,  // MDMCFG0  - Modem configuration.
    0x0a, 0x00,  // CHANNR   - Channel number.
    0x15, 0x62,  // DEVIATN  - Modem deviation setting (when FSK modulation is
                 // enabled).
    0x21, 0xb6,  // FREND1   - Front end RX configuration.
    0x22, 0x10,  // FREND0   - Front end RX configuration.
    0x18, 0x18,  // MCSM0    - Main Radio Control State Machine configuration.
    0x17, 0x00,  // MCSM1    - Main Radio Control State Machine configuration.
    0x19, 0x1d,  // FOCCFG   - Frequency Offset Compensation Configuration.
    0x1a, 0x1c,  // BSCFG    - Bit synchronization Configuration.
    0x1b, 0xc7,  // AGCCTRL2 - AGC control.
    0x1c, 0x00,  // AGCCTRL1 - AGC control.
    0x1d, 0xb0,  // AGCCTRL0 - AGC control.
    0x07, 0x4c,  // PKTCTRL1
    0x08, 0x05,  // PKTCTRL0
    0x06, 0x3e,  // PKTLEN
    //    0x23, 0xea,    // FSCAL3   - Frequency synthesizer calibration.
    //    0x24, 0x2a,    // FSCAL2   - Frequency synthesizer calibration.
    //    0x25, 0x00,    // FSCAL1   - Frequency synthesizer calibration.
    //    0x26, 0x1f,    // FSCAL0   - Frequency synthesizer calibration.
    //    0x29, 0x59,    // FSTEST   - Frequency synthesizer calibration.
    //    0x2c, 0x88,    // TEST2    - Various test settings.
    //    0x2d, 0x31,    // TEST1    - Various test settings.
    //    0x2e, 0x09,    // TEST0    - Various test settings.
    0x03, 0x07,  // FIFOTHR  - RXFIFO and TXFIFO thresholds.
    //    0x00, 0x0b,    // IOCFG2   - GDO2 output pin configuration.
    //    0x02, 0x0c,    // IOCFG0D  - GDO0 output pin configuration. Refer to
    //    SmartRF� Studio User Manual for detailed pseudo register explanation.
    //    0x07, 0x04,    // PKTCTRL1 - Packet automation control.
    //    0x08, 0x12,    // PKTCTRL0 - Packet automation control.
    //    0x09, 0x00,    // ADDR     - Device address.
    //    0x06, 0xff,    // PKTLEN   - Packet length.
};

void host_main() {
    cli();

    // Set up USB
    clear_bit(UHWCON, UIMOD);

    set_bit(OTGCON, VBUSHWC);
    set_bit(PORTE, 7);

    clear_bit(UDCON, DETACH);
    set_bit(USBCON, HOST);

    OTGIEN = (1 << SRPE);
    UHIEN = (1 << DCONNE) | (1 << DDISCE) | (1 << RSTE);

    // Setup timer 0 for sending forward packets at about 60Hz
    TCCR0A = 0x02;  // CTC mode
    TCCR0B = 0x05;  // clk_io/1024, CTC mode
    OCR0A = 129;    // 130 counts/overflow
    set_bit(TIMSK0, OCIE0A);

    // Initialize radio
    radio_select();
    for (uint8_t i = 0; i < sizeof(cc1101_regs); ++i) {
        spi_write(pgm_read_byte(&cc1101_regs[i]));
    }
    radio_deselect();

    // Configure external interrupt 6 (GDO2 from radio)
    radio_write(IOCFG2, 6 | GDOx_INVERT);
    set_bit(EIFR, 2);

    sei();

    printf_P(PSTR("Initialized\n"));

    sleep_enable();
    while (1) {
        sleep_cpu();

        if (usb_state == USB_Connected) {
            // Need to configure outside an interrupt because this can take a
            // long time.
            if (configure_device()) {
                usb_state = USB_Configured;
            } else {
                usb_state = USB_Broken;
            }
        }
    }
}
