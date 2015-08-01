#if defined(TARGET_LPC1768) || defined(TARGET_LPC1114) || defined(TARGET_LPC11U24)
#include "BurstSPI.h"

void BurstSPI::fastWrite(int data) {
    //Wait until FIFO has space
    while(((_spi.spi->SR) & 0x02) == 0);
    
    //transmit data
    _spi.spi->DR = data;
    }

void BurstSPI::clearRX( void ) {
    //Do it while either data in RX buffer, or while it is busy
    while(((_spi.spi->SR) & ((1<<4) + (1<<2))) != 0) {
        //Wait until data in RX buffer
        while(((_spi.spi->SR) & (1<<2)) == 0);
        int dummy = _spi.spi->DR;
        }
}
#endif