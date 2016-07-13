#include "i2c.h"

#include <board.h>

void i2c_init() {
    const int Fi2c = 400000;
    const int TotalDiv = BOARD_MCK / Fi2c;
    const int CKDIV = 0;
    const int CxDIV = (TotalDiv - 3) / (1 << CKDIV);

    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_TWI;
    AT91C_BASE_TWI->TWI_CWGR = (CKDIV << 16) | (CxDIV << 8) | (CxDIV);
    AT91C_BASE_TWI->TWI_CR = AT91C_TWI_MSEN;
    AT91C_BASE_PIOA->PIO_PDR = I2C_SDA | I2C_SCL;
    AT91C_BASE_PIOA->PIO_MDER = I2C_SDA | I2C_SCL;
    AT91C_BASE_PIOA->PIO_ASR = I2C_SDA | I2C_SCL;
}
