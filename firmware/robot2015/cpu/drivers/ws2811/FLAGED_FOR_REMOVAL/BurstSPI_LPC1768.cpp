#if defined(TARGET_LPC1768) || defined(TARGET_LPC1114) || defined(TARGET_LPC11U24)

#include "BurstSPI.hpp"

void BurstSPI::fastWrite(unsigned int data)
{
    //Wait until TX FIFO has space
    while (((SSP_BASE->SSP_SR) & 0x02) == 0);

    //transmit data
    SSP_BASE->SSP_DR = data;
}

void BurstSPI::clearRX(void)
{
    uint32_t dummy;

    //Do it while either data in RX buffer, or while it is busy
    while (((SSP_BASE->SSP_SR) & ((1 << 4) | (1 << 2))) != 0) {
        //Wait until data in RX buffer
        while (((SSP_BASE->SSP_SR) & (1 << 2)) == 0);

        dummy = SSP_BASE->SSP_DR;
    }
}

#endif
