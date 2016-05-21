#pragma once

#include <stdint.h>
#include <board.h>

void spi_init(void);
void spi_shutdown(void);

// Transfers one byte
uint8_t spi_xfer(uint8_t tx);

// Selects SPI device 0-3
void spi_select(int n);

// Deselects the current SPI device.
// If a transfer is in progress, the device will be deselected after the
// transfer is finished.
// Another transfer to the same device can be started without calling
// spi_select() again.
static inline void spi_deselect() {
    AT91C_BASE_SPI->SPI_CR = AT91C_SPI_LASTXFER;
}
