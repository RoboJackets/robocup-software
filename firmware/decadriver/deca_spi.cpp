/*! ----------------------------------------------------------------------------
 * @file	deca_spi.c
 * @brief	SPI access functions
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#include <string.h>

#include "deca_spi.h"
#include "deca_sleep.h"
#include "deca_device_api.h"

#include "mbed.h"

int writetospi(uint16 headerLength, const uint8 *headerBuffer,
                      uint32 bodylength, const uint8 *bodyBuffer);

int readfromspi(uint16 headerLength, const uint8 *headerBuffer,
                       uint32 readlength, uint8 *readBuffer);
/*!
 *------------------------------------------------------------------------------------------------------------------
 * Function: openspi()
 *
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(/*SPI_TypeDef* SPIx*/) {
  DigitalOut tmp_ncs(p10, 0);

  return 0;

} // end openspi()

/*!
 *------------------------------------------------------------------------------------------------------------------
 * Function: closespi()
 *
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void) {
  DigitalOut tmp_ncs(p10, 1);

  return 0;

} // end closespi()

/*!
 *------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */
#pragma GCC optimize("O3")
int writetospi(uint16 headerLength, const uint8 *headerBuffer,
                      uint32 bodylength, const uint8 *bodyBuffer) {
  SPI _spi(p5, p6, p7);
  //_spi.format(8,0);
  //_spi.frequency(2000000);

  // decaIrqStatus_t  stat = decamutexon();

  DigitalOut tmp_ncs(p10, 0);

  for (size_t i = 0; i < headerLength; i++)
    _spi.write(headerBuffer[i]);

  for (size_t i = 0; i < bodylength; i++)
    _spi.write(bodyBuffer[i]);

  tmp_ncs = 1;

  // decamutexoff(stat);

  return 0;
} // end writetospi()

/*!
 *------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be
 *found,
 * or returns -1 if there was an error
 */
#pragma GCC optimize("O3")
int readfromspi(uint16 headerLength, const uint8 *headerBuffer,
                       uint32 readlength, uint8 *readBuffer) {
  SPI _spi(p5, p6, p7);
  //_spi.format(8,0);
  //_spi.frequency(2000000);

  // decaIrqStatus_t  stat = decamutexon() ;

  /* Wait for SPIx Tx buffer empty */
  // while (port_SPIx_busy_sending());

  DigitalOut tmp_ncs(p10, 0);
  //Serial pc(USBTX,USBRX);

  for (size_t i = 0; i < headerLength; i++) {
    readBuffer[0] = _spi.write(headerBuffer[i]);
    //pc.printf("%d",headerBuffer[i]);
  }

  for (size_t i = 0; i < readlength; i++) {
    readBuffer[i] = _spi.write(0);
    //pc.printf("%d",0);
  }
  //pc.printf("\n");
  //wait_ms(50);


  tmp_ncs = 1;

  // decamutexoff(stat);
  //pc.printf(readBuffer);

  return 0;
} // end readfromspi()
