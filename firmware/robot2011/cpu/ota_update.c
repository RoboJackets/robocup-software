#include "ota_update.h"
#include "radio.h"
#include "spi.h"
#include "timer.h"
#include "main.h"
#include "status.h"
#include "radio_protocol.h"

// First page used for the temporary copy
#define MIRROR_PAGE (AT91C_IFLASH_NB_OF_PAGES / 2)

// Amount of data received so far
static uint32_t ota_data_len;

// If nonzero, we have finished receiving data and are ready to copy it after
// sending one last reply
static int ota_commit;

// Start of last page that was written to flash
static uint32_t last_page_start;

__attribute__((section(".reflash"))) void flash_write_page(int page) {
    uint32_t old_ints = AT91C_BASE_AIC->AIC_IMR;
    AT91C_BASE_AIC->AIC_IDCR = ~0;

    AT91C_BASE_MC->MC_FCR = 0x5a000000 | (page << 8) | AT91C_MC_FCMD_START_PROG;
    while (!(AT91C_BASE_MC->MC_FSR & AT91C_MC_FRDY)) {
        AT91C_BASE_WDTC->WDTC_WDCR = 0xa5000001;
    }

    AT91C_BASE_AIC->AIC_IECR = old_ints;
}

__attribute__((section(".reflash"))) void ota_copy(int num_pages) {
    AT91C_BASE_AIC->AIC_IDCR = ~0;

    LED_OFF(LED_ALL);
    LED_ON(LED_RG);

    uint32_t* dest = (uint32_t*)AT91C_IFLASH;
    for (int page = 0; page < num_pages; ++page) {
        const uint32_t* src =
            (const uint32_t*)(AT91C_IFLASH +
                              AT91C_IFLASH_PAGE_SIZE * (page + MIRROR_PAGE));
        for (int i = 0; i < AT91C_IFLASH_PAGE_SIZE / 4; ++i) {
            dest[i] = src[i];
        }

        AT91C_BASE_MC->MC_FCR =
            0x5a000000 | (page << 8) | AT91C_MC_FCMD_START_PROG;
        while (!(AT91C_BASE_MC->MC_FSR & AT91C_MC_FRDY)) {
            AT91C_BASE_WDTC->WDTC_WDCR = 0xa5000001;
        }

        LED_TOGGLE(LED_RR);
    }
    LED_OFF(LED_RG);

    // Reset processor and peripherals
    *AT91C_RSTC_RCR = 0xa5000005;
    while (1)
        ;
}

// Handles a forward packet in OTA mode.
// Returns nonzero if OTA mode should continue.
static int ota_packet() {
    rx_lost_time = current_time;

    LED_TOGGLE(LED_RG);

    // Go ahead and store our expected offset in the reply
    reply_len = 0;
    reply_buf[1] = ota_data_len;
    reply_buf[2] = ota_data_len >> 8;
    reply_buf[3] = ota_data_len >> 16;

    rx_lost_time = current_time;
    // Special commands
    if (radio_rx_len == 2 && radio_rx_buf[0] == 0xc9 &&
        radio_rx_buf[1] == 0xd5) {
        // (Re)start.
        ota_data_len = 0;
        last_page_start = 0;
        ota_commit = 0;

        reply_buf[0] = robot_id;
        reply_len = 1;
    } else if (radio_rx_len == 4 && radio_rx_buf[0] == 0x6b) {
        // Finish
        reply_len = 4;
        uint32_t offset =
            radio_rx_buf[1] | (radio_rx_buf[2] << 8) | (radio_rx_buf[3] << 16);
        if (offset > ota_data_len) {
            // Missed data
            reply_buf[0] = robot_id | 0x40;
        } else if (offset < ota_data_len) {
            // Duplicate data
            reply_buf[0] = robot_id | 0x80;
        } else {
            // Write the last page if necessary
            if (ota_data_len != last_page_start) {
                int page = ota_data_len / AT91C_IFLASH_PAGE_SIZE +
                           AT91C_IFLASH_NB_OF_PAGES / 2;
                flash_write_page(page);
            }

            ota_commit = 1;
            reply_buf[0] = robot_id | 0xf0;
        }
    } else if (radio_rx_len == 2 && radio_rx_buf[0] == 0x00 &&
               radio_rx_buf[1] == 0xff) {
        // Abort
        return 0;
    } else if ((radio_rx_len & 3) == 0 && (radio_rx_buf[0] & 0xfe) == 0x38) {
        uint32_t offset =
            radio_rx_buf[1] | (radio_rx_buf[2] << 8) | (radio_rx_buf[3] << 16);
        if (radio_rx_buf[0] & 0x01) {
            reply_len = 4;
        }
        if (offset > ota_data_len) {
            // Missed data
            reply_buf[0] = robot_id | 0x40;
        } else if (offset < ota_data_len) {
            // Duplicate data
            reply_buf[0] = robot_id | 0x80;
        } else {
            // New data
            // FIXME - Copy data only up to the next page boundary and update
            // last_page_start to match
            // Add to the buffer and write to flash if we have finished another
            // page
            uint32_t len = radio_rx_len - 4;
            const uint32_t* src = (const uint32_t*)(radio_rx_buf + 4);
            uint32_t* dest = (uint32_t*)(AT91C_IFLASH + AT91C_IFLASH_SIZE / 2 +
                                         ota_data_len);
            for (int i = 0; i < len / 4; ++i) {
                dest[i] = src[i];
            }

            // If this packet puts us at or past the end of another page, write
            // it
            if ((ota_data_len + len - last_page_start) >=
                AT91C_IFLASH_PAGE_SIZE) {
                int page = last_page_start / AT91C_IFLASH_PAGE_SIZE +
                           AT91C_IFLASH_NB_OF_PAGES / 2;
                flash_write_page(page);
                last_page_start = ota_data_len + len;
            }

            ota_data_len += len;
            reply_buf[0] = robot_id;
        }
    }

    // If we are sending a reply, start the reply timer for the proper time slot
    if (reply_len) {
        // 2ms per slot
        reply_timer_start(3000 * robot_id);
    } else {
        radio_command(SRX);
    }

    return 1;
}

static void ota_main() {
    LED_ON(LED_ALL);
    LED_OFF(LED_RY);

    AT91C_BASE_MC->MC_FMR = 0x00340100;

    int lost_radio_count = 0;

    // Reset state by processing the packet that got us here
    ota_packet();

    while (1) {
        // Reset the watchdog timer
        AT91C_BASE_WDTC->WDTC_WDCR = 0xa5000001;

        // Keep the radio working
        if ((current_time - rx_lost_time) > 250) {
            rx_lost_time = current_time;
            LED_OFF(LED_RG);
            LED_TOGGLE(LED_RR);

            ++lost_radio_count;
            if (lost_radio_count == 10) {
                lost_radio_count = 0;
                radio_configure();
            } else {
                radio_command(SIDLE);
                radio_command(SFRX);
                radio_command(SRX);
            }
        }

        // Check for radio data
        if (radio_poll()) {
            if (!ota_packet()) {
                return;
            }
        }

        // Send a reply in the proper time slot
        radio_reply();

        // Copy data and reset after sending a final reply to the FINISH command
        if (ota_commit && reply_len == 0 && !radio_in_tx) {
            // Copy flash and reset
            ota_copy((ota_data_len + AT91C_IFLASH_PAGE_SIZE - 1) /
                     AT91C_IFLASH_PAGE_SIZE);
            return;
        }
    }
}

int ota_start() {
    if (radio_rx_len == 2 && radio_rx_buf[0] == 0xc9 &&
        radio_rx_buf[1] == 0xd5) {
        ota_main();
        return 1;
    }

    return 0;
}
