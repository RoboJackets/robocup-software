    

/* BurstSPI_NUCLEO_L152RE.cpp */
#ifdef TARGET_NUCLEO_L152RE
#include "BurstSPI.h"
 
void BurstSPI::fastWrite(int data) {
    
    SPI_TypeDef *spi = (SPI_TypeDef *)(_spi.spi);
    // Check if data is transmitted
    while (!((SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_TXE) != RESET) ? 1 : 0));
    SPI_I2S_SendData(spi, (uint16_t)data);
    }
    
void BurstSPI::clearRX( void ) {
    int status;
    //Check if the RX buffer is busy
    SPI_TypeDef *spi = (SPI_TypeDef *)(_spi.spi);
    status = ((SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_BSY) != RESET) ? 1 : 0);
    if (status){   
        // Check RX buffer readable
        while (!((SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_RXNE) != RESET) ? 1 : 0));
        int dummy = (int)SPI_I2S_ReceiveData(spi);
    }
}
#endif

