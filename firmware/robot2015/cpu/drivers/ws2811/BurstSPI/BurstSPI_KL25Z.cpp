#ifdef TARGET_KL25Z
#include "BurstSPI.h"

void BurstSPI::fastWrite(int data) {
    //Wait until FIFO has space
    while(((_spi.spi->S) & SPI_S_SPTEF_MASK) == 0);
    //transmit data
    _spi.spi->D = data;
    }

void BurstSPI::clearRX( void ) {
    //We put in a delay here, this function shouldn't be called very often, so not a huge problem
    //Without delay you will rise the CS line before it is finished (been there, done that)
    //We use time required to transmit 20 bits (8 bits being transmitted, 8 bits in FIFO, 4 bits safety margin
    
    float bytetime = 20.0/_hz;
    wait(bytetime);    
    
    //Wait until status is flagged that we can read, read:
    while (_spi.spi->S & SPI_S_SPRF_MASK == 0);
    int dummy = _spi.spi->D;

}
#endif