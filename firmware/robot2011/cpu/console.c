#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <board.h>
#include <stdio.h>
#include <string.h>

#include "console.h"
#include "write.h"
#include "main.h"
#include "control.h"

unsigned char usb_rx_buffer[64];
// Amount of data in usb_rx_buffer.
// This is set by the RX callback (called from an interrupt)
// and cleared after the data is handled by the main loop.
volatile int usb_rx_len;

unsigned char usb_tx_buffer[64];
int usb_tx_len;

char command_buffer[MAX_COMMAND_SIZE];
int command_len;

void subcommand(int argc, const char* argv[], void* arg) {
    const command_t* list = (const command_t*)arg;

    // Find the command
    for (int i = 0; list[i].name; ++i) {
        if (!strcmp(argv[0], list[i].name)) {
            list[i].handler(argc - 1, argv + 1, list[i].arg);
            return;
        }
    }

    printf("*** Unrecognized command\n");
}

void console_execute_buffer() {
    const char* argv[MAX_COMMAND_ARGS];
    int argc;

    // Split arguments
    char* line = command_buffer;
    for (argc = 0;; ++argc) {
        // Skip leading whitespace
        while (1) {
            if (*line == 0) {
                goto split_done;
            }
            if (*line != ' ') {
                break;
            }
            ++line;
        }

        if (argc == MAX_COMMAND_ARGS) {
            printf("*** Too many arguments\n");
            return;
        }

        // Store the beginning of this word
        argv[argc] = line;

        // Find the end of this word
        while (*line != ' ') {
            if (*line == 0) {
                // End of the last word
                ++argc;
                goto split_done;
            }
            ++line;
        }
        *line++ = 0;
    }
split_done:

    if (argc) {
        subcommand(argc, argv, (void*)commands);
    }
}

void flush_stdout() {
    // This must not be called from within a USB callback
    // because the write function will not succeed until the interrupt
    // at the end of the last TX is serviced.
    while (CDCDSerialDriver_Write(usb_tx_buffer, usb_tx_len, 0, 0) !=
           USBD_STATUS_SUCCESS)
        ;
    usb_tx_len = 0;
}

int putchar(int c) {
    if (c == '\n') {
        putchar_raw('\r');
    }
    putchar_raw(c);
    if (c == '\n') {
        flush_stdout();
    }
    return c;
}

void putchar_raw(uint8_t c) {
    usb_tx_buffer[usb_tx_len++] = c;
    if (usb_tx_len == sizeof(usb_tx_buffer)) {
        flush_stdout();
    }
}

int puts(const char* str) {
    while (*str) {
        putchar(*str++);
    }
    putchar('\n');

    return 0;
}

static void print_prompt() {
    putchar('>');
    putchar(' ');
    flush_stdout();
}

void usb_rx_complete(void* arg, unsigned char status, unsigned int received,
                     unsigned int remaining) {
    if (status == USBD_STATUS_SUCCESS) {
        usb_rx_len = received;
    }
}

void usb_rx_start() {
    usb_rx_len = 0;
    CDCDSerialDriver_Read(usb_rx_buffer, sizeof(usb_rx_buffer), usb_rx_complete,
                          0);
}

void console_init() {
    command_len = 0;

    // Print the first command prompt
    // 	print_prompt();

    // Start reading commands
    usb_rx_start();
}

int console_run() {
    int ret = 1;

    if (usb_rx_len) {
        for (int i = 0; i < usb_rx_len; ++i) {
            char ch = usb_rx_buffer[i];
            if (ch == 3) {
                printf("Stop\n");
                debug_update = 0;
                ret = 0;

                // Clear the command buffer
                command_len = 0;
            } else if (ch == '\r') {
                putchar('\n');
                if (command_len >= MAX_COMMAND_SIZE) {
                    // Command too long
                    printf("Command too long\n");
                } else {
                    // We still have room for the terminator
                    command_buffer[command_len] = 0;

                    // Execute the command
                    console_execute_buffer();
                }

                // Clear the command buffer
                command_len = 0;
            } else if (ch == 8 || ch == 0x7f) {
                // Backspace
                if (command_len) {
                    putchar(8);
                    putchar(32);
                    putchar(8);
                    flush_stdout();
                    --command_len;
                }
                continue;
            } else {
                if (ch >= 0x20 && ch <= 0x7e &&
                    command_len < MAX_COMMAND_SIZE) {
                    // Add to command buffer
                    command_buffer[command_len++] = ch;
                    putchar(ch);
                    flush_stdout();
                }
                continue;
            }
            print_prompt();
        }

        // Clear the buffer and keep reading
        usb_rx_start();
    }

    return ret;
}

uint32_t parse_uint32(const char* str) {
    uint32_t value = 0;

    if (str[0] == '0' && str[1] == 'x') {
        // Hexadecimal
        str += 2;
        while (*str != 0) {
            char ch = *str++;

            value *= 16;

            if (ch >= '0' && ch <= '9') {
                value += ch - '0';
            } else if (ch >= 'a' && ch <= 'f') {
                value += ch - 'a' + 10;
            } else if (ch >= 'A' && ch <= 'F') {
                value += ch - 'A' + 10;
            } else {
                break;
            }
        }
    } else {
        // Decimal
        while (*str != 0) {
            char ch = *str++;

            value *= 10;

            if (ch >= '0' && ch <= '9') {
                value += ch - '0';
            } else {
                break;
            }
        }
    }

    return value;
}

int parse_int(const char* str) {
    int32_t value = 0;
    int negative = 0;

    if (str[0] == '-') {
        negative = 1;
        ++str;
    }

    if (str[0] == '0' && str[1] == 'x') {
        // Hexadecimal
        str += 2;
        while (*str != 0) {
            char ch = *str++;

            value *= 16;

            if (ch >= '0' && ch <= '9') {
                value += ch - '0';
            } else if (ch >= 'a' && ch <= 'f') {
                value += ch - 'a' + 10;
            } else if (ch >= 'A' && ch <= 'F') {
                value += ch - 'A' + 10;
            } else {
                break;
            }
        }
    } else {
        // Decimal
        while (*str != 0) {
            char ch = *str++;

            value *= 10;

            if (ch >= '0' && ch <= '9') {
                value += ch - '0';
            } else {
                break;
            }
        }
    }

    if (negative) {
        value = -value;
    }

    return value;
}

// Detects and handles USB connection/disconnection
enum {
    USB_Disconnected,
    USB_Connected,
    USB_Working
} usb_state = USB_Disconnected;

void check_usb_connection() {
    int vbus = AT91C_BASE_PIOA->PIO_PDSR & VBUS;
    if (vbus && usb_state == USB_Disconnected) {
        // Initialize the USB controller.
        // This will disconnect from the host (important if we have just reset
        // after reprogramming).
        // The host will notice disconnection while we check the FPGA below.
        CDCDSerialDriver_Initialize();
        USBD_Connect();
        usb_state = USB_Connected;
    }
    if (!vbus) {
        usb_state = USB_Disconnected;
    }

    if (usb_state == USB_Connected &&
        USBD_GetState() >= USBD_STATE_CONFIGURED) {
        console_init();
        usb_state = USB_Working;
    }

    if (usb_state == USB_Working) {
        if (!console_run()) {
            controller = 0;
        }
    }
}

int usb_is_connected(void) { return usb_state == USB_Working; }
