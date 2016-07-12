#include <stdint.h>
#include <board.h>

#include "csr.h"
#include "reflash.h"

#define EP_OUT 1
#define EP_IN 2

#define wdt_reset() AT91C_BASE_WDTC->WDTC_WDCR = 0xa5000001;

__attribute__((section(".reflash"))) int reflash_read(uint8_t* ptr) {
    int i;
    int n = AT91C_UDP_CSR[EP_OUT] >> 16;
    for (i = 0; i < n; i += 4) {
        uint8_t b0 = AT91C_UDP_FDR[EP_OUT];
        uint8_t b1 = AT91C_UDP_FDR[EP_OUT];
        uint8_t b2 = AT91C_UDP_FDR[EP_OUT];
        uint8_t b3 = AT91C_UDP_FDR[EP_OUT];

        *(uint32_t*)ptr = b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
        ptr += 4;
    }

    return n;
}

// Entry point
// MC_FMR must already be set for flash operations.  USB must be configured, but
// RX bank doesn't matter.
// There must be no pending USB RX data except for the first packet of flash
// data
// (this is necessary to allow the current RX bank to be detected).
__attribute__((section(".reflash"))) void reflash_main(int total_size) {
    AT91C_BASE_PIOA->PIO_OER = LED_ALL;
    AT91C_BASE_PIOA->PIO_SODR = LED_ALL;

    uint8_t* start = (uint8_t*)AT91C_IFLASH;
    uint8_t* ptr = start;

    // Number of next OUT bank to read
    int bank;
    uint32_t csr;
    int page, new_page;

    AT91C_BASE_PIOA->PIO_CODR = LED_LY;
    // Wait for an RX packet and determine which bank it's in.
    // If two packets are available (both bits are set), this will fail and
    // there is no way to recover.  We must reach this point soon enough after
    // starting
    // the flash procedure that there is no chance of two packets having
    // arrived.
    while (!((csr = AT91C_UDP_CSR[EP_OUT]) &
             (AT91C_UDP_RX_DATA_BK0 | AT91C_UDP_RX_DATA_BK1)))
        wdt_reset();
    if (csr & AT91C_UDP_RX_DATA_BK0) {
        bank = 0;
    } else {
        bank = 1;
    }

    AT91C_BASE_PIOA->PIO_CODR = LED_LR;
    // Read data from USB and write to flash.
    page = ((uint32_t)ptr & 0xffff) / AT91C_IFLASH_PAGE_SIZE;
    for (ptr = start; ptr < start + total_size;) {
        // Read one packet from the current bank
        // FIXME - The page buffer wraps around and nothing here prevents
        // overwriting earlier data
        if (bank == 0) {
            while (!(AT91C_UDP_CSR[EP_OUT] & AT91C_UDP_RX_DATA_BK0))
                wdt_reset();
            ptr += reflash_read(ptr);
            CLEAR_CSR(1, AT91C_UDP_RX_DATA_BK0);
        } else {
            while (!(AT91C_UDP_CSR[EP_OUT] & AT91C_UDP_RX_DATA_BK1))
                wdt_reset();
            ptr += reflash_read(ptr);
            CLEAR_CSR(1, AT91C_UDP_RX_DATA_BK1);
        }

        // Switch banks
        bank ^= 1;

        // If this data has run over a page boundary, write the old page.
        // The page size is either 128 or 256 bytes while the max packet size is
        // 64 bytes,
        // so we will only ever need to write one page here.
        new_page = ((uint32_t)ptr & 0xffff) / AT91C_IFLASH_PAGE_SIZE;
        if (page != new_page) {
            *AT91C_MC_FCR = 0x5a000000 | (page << 8) | AT91C_MC_FCMD_START_PROG;
            while (!(*AT91C_MC_FSR & AT91C_MC_FRDY)) wdt_reset();
            page = new_page;
            AT91C_BASE_PIOA->PIO_CODR = LED_RR;
        }
    }

    AT91C_BASE_PIOA->PIO_CODR = LED_RY;
    // Send data back to verify
    while (AT91C_UDP_CSR[EP_IN] & AT91C_UDP_TXPKTRDY) wdt_reset();
    CLEAR_CSR(2, AT91C_UDP_TXCOMP);
    for (ptr = start; ptr < start + total_size;) {
        int i;

        for (i = 0; i < 64; ++i) {
            AT91C_UDP_FDR[EP_IN] = *ptr++;
        }
        SET_CSR(2, AT91C_UDP_TXPKTRDY);

        while (!(AT91C_UDP_CSR[EP_IN] & AT91C_UDP_TXCOMP)) wdt_reset();
        CLEAR_CSR(2, AT91C_UDP_TXCOMP);
    }
    AT91C_BASE_PIOA->PIO_CODR = LED_RG;

    // Reset processor and peripherals
    *AT91C_RSTC_RCR = 0xa5000005;
    while (1)
        ;
}
