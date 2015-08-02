#if 0

#if defined(TARGET_LPC1768) || defined(TARGET_LPC1114) || defined(TARGET_LPC11U24)

#include "BurstSPI.hpp"

namespace BurstSPI
{

void BurstSPI::fastWrite(unsigned int data) {
    //Wait until TX FIFO has space
    while (((_spi.spi->SSP_SR) & 0x02) == 0);

    //transmit data
    _spi.spi->SSP_DR = data;
}

void BurstSPI::clearRX(void) {
    uint32_t dummy;

    //Do it while either data in RX buffer, or while it is busy
    while (((_spi.spi->SSP_SR) & ((1 << 4) | (1 << 2))) != 0) {
        //Wait until data in RX buffer
        while (((_spi.spi->SSP_SR) & (1 << 2)) == 0);

        dummy = _spi.spi->SSP_DR;
    }
}

}   // namespace

#endif

#endif  // #if 0
