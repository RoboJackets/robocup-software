#include "adc-dma.hpp"

#define SystemFrequency 1000000

uint8_t ADCDMA::dmaChannelNum = 0;


/**
 *
 */
ADCDMA::ADCDMA(void)
{
  isInit = false;
  ADC_int_done = true;
  burstEn = false;

  BurstCounter = 0;
  overrun_count = 0;
  channel_flag = 0;
  ADCIntDone = 0;

  // Clear out a section of memory for storing the ADC readings
  memset(adc_buf, 0, sizeof(adc_buf));
  memset(dmaTransferComplete, 0, sizeof(dmaTransferComplete));

  if (Init(ADC_CLK))
    log(INF1, "ADC DMA", "ADC using DMA setup complete.");
}


/**
 *
 */
ADCDMA::~ADCDMA(void)
{
  if (shutdown())
    log(INF3, "ADC DMA", "ADC DMA killed. All ADC interrupts removed.");
}


/**
 * [ADCDMA::Init Setup the FPGA interface]
 * @return  [The initialization error code.]
 */


//ERR_t ADCDMA::Init(void)
//{
//  if (isInit)
//return 0;

// ADCStruct.adcRate = ADC_SAMPLE_RATE;
// ADCStruct.burstMode = Burst_Mode_Flag;
// ADCStruct.bitsAccuracy = ADC_10BITS;

// Chip_ADC_Init(_LPC_ADC_ID, &ADCStruct);

// Enable all of the ADC channels
// for (int i = 0; i < ADC_NUM_CHANNELS_CHANNELS; i++)
// Chip_ADC_Channel_Int_Cmd(_LPC_ADC_ID, ADC_INT_TRIGGER_CHAN, ENABLE);

// dmaChannelNum = Chip_DMA_GetFreeChannel(LPC_GPDMA, _GPDMA_CONN_ADC);
// log(INF2, "ADC DMA", "ADC setup complete. Now linking requests to DMA channel %u.", dmaChannelNum);

// Power up the ADC and set PCLK
// LPC_SC->PCONP    |=  (1UL << 12);
// LPC_SC->PCLKSEL0 &= ~(3UL << 24); // 10biPCLK = CCLK/4 96M/4 = 24MHz

// Enable the ADC, 12MHz,  ADC0.0 & .1
// _LPC_ADC_ID->ADCR  = (1UL << 21) | (1UL << 8) | (3UL << 0);

// FIX ME - put the correct pins ==========
// Set the pin functions to ADC
/*
LPC_PINCON->PINSEL1 &= ~(3UL << 14);  // P0.23, Mbed p15.
LPC_PINCON->PINSEL1 |=  (1UL << 14);
LPC_PINCON->PINSEL1 &= ~(3UL << 16);  // P0.24, Mbed p16.
LPC_PINCON->PINSEL1 |=  (1UL << 16);
*/

// Prepare an ADC configuration.
/*
MODDMA_Config *conf = new MODDMA_Config;
conf
->channelNum    ( MODDMA::Channel_0 )
->srcMemAddr    ( 0 )
->dstMemAddr    ( (uint32_t)adc_buf )
->transferSize  ( SAMPLE_BUFFER_LENGTH )
->transferType  ( MODDMA::p2m )
->transferWidth ( MODDMA::word )
->srcConn       ( MODDMA::ADC )
->dstConn       ( 0 )
->dmaLLI        ( 0 )
->attach_tc     ( &DMA_IRQHandler )
                ( &ADC_IRQHandler )
->attach_err    ( &ERR0_callback )
; // end conf.
*/

// Prepare configuration.
// dma.Setup( conf );

// Enable configuration.
// dma.Enable( conf );
// log(INF1, "ADC DMA", "ADC using DMA setup complete.");

// Enable ADC irq flag (to DMA).
// Note, don't set the individual flags,
// just set the global flag.
// _LPC_ADC_ID->ADINTEN = 0x100;

// Enable burst mode on inputs 0 and 1.
// _LPC_ADC_ID->ADCR |= (1UL << 16);
// isInit = true;

// return 0;
// }


/**
 * [ADCDMA::Check Poll a DMA transfer for completion.]
 * @return  [Returns TRUE when the DMA transfer was complete & the function retreived the values.]
 */
bool ADCDMA::Poll(void)
{
  // When transfer complete do this block.
  if (dmaTransferComplete[0]) {
    dmaTransferComplete[0] = false;

    // Read all ADCs
    for (int i = 0; i < ADC_NUM_CHANNELS; i++)
      log(INF2, "ADC DMA", "Polled reading value:\t%u", ADC_GET_READING(adc_buf[i][0]));

    for (int i = 0; i < ADC_NUM_CHANNELS; i++) {
      // return TRUE is ANY channel reading is obtained
      if (adc_buf[i][0] != 0)
        return true;
    }
  }

  return false;
}


/**
 * [ADCDMA::InterruptTest description]
 */
bool ADCDMA::InterruptTest(void)
{
  /* Enable ADC Interrupt */
  NVIC_EnableIRQ(_LPC_ADC_IRQ);
  // Chip_ADC_Channel_Int_Cmd(_LPC_ADC_ID, ADC_INT_TRIGGER_CHAN, ENABLE);

  /* Enable burst mode if any, the AD converter does repeated conversions
     at the rate selected by the CLKS field in burst mode automatically */
  ADC_burst_on();

  bool Interrupt_Continue_Flag = true;
  ADC_int_done = true;

  while (Interrupt_Continue_Flag) {
    if (!burstEn && ADC_int_done)
      ADC_int_done = false;
  }

  /* Disable burst mode if any */
  // if (Burst_Mode_Flag)
  // Chip_ADC_Burst_Cmd(_LPC_ADC_ID, DISABLE);

  NVIC_DisableIRQ(_LPC_ADC_IRQ);

  return true;
}


/**
 * [ADCDMA::DMA_IRQHandler description]
 */
void ADCDMA::DMA_IRQHandler(void)
{
  // if (Chip_DMA_Interrupt(LPC_GPDMA, ADCDMA::dmaChannelNum) == SUCCESS) {
  // channelTC++;
  // } else {
  /* Process error here */

  // turn off burst and disable
  if (0) {
    ADC_burst_off();
    ADC_stop();
  }

  log(SEVERE, "ADC DMA", "Unable to successfully configure the DMA for use with the ADC pins.");
  // }
}


/**
 * [ADCDMA::Shutdown description]
 * @return  [description]
 */
bool ADCDMA::shutdown(void)
{
  NVIC_DisableIRQ(_LPC_ADC_IRQ);
  ADC_burst_off();
  ADC_stop();
  isInit = false;

  // Successfully cleaned everything up
  return true;
}


/**
 * [ADC_IRQHandler  description]
 */
void ADCDMA::ADC_IRQHandler (void)
{
  // uint32_t dummy;
  uint32_t regVal;

  // Reading the ADC will clear the interrupt
  regVal = ((_LPC_ADC_ID->ADSTAT) >> 8) & 0x7F;

  // check OVERRUN error first
  if (regVal) {
    overrun_count++;

    for (int i = 0; i < ADC_NUM_CHANNELS; i++) {
      uint8_t chan_overrun = (regVal >> i);

      /* if overrun, just read ADDR to clear */
      /* regVal variable has been reused. */
      if (chan_overrun)
        uint32_t dummy = _LPC_ADC_ID->ADDR0 + i;
    }

    ADC_stop();
    ADC_burst_off();
    ADC_int_done = 1;
  }

  for (int i = 0; i < ADC_NUM_CHANNELS; i++)
    if (regVal & _BV(i))
      adc_buf[i][0] = ADC_GET_READING(_LPC_ADC_ID->ADDR0 + i);

#if ADC_BURST_MODE
  BurstCounter++;
  channel_flag |= (regVal & 0xFF);

  if ((channel_flag & 0xFF) == 0xFF) {
    ADC_burst_off();
    channel_flag = 0;
    ADCIntDone = 1;
  }

#else
// _LPC_ADC_ID->ADCR &= ~(0x7 << 24);  /* stop ADC now */
  ADC_stop();
  ADCIntDone = 1;
#endif
}


/**
 * [ADCDMA::Offset Retrieve the ADC offset value set by the bootloader at startup.]
 * @return  [ADC offset value]
 */
uint8_t ADCDMA::Offset(void)
{
  return ADC_GET_READING(_LPC_ADC_ID->ADTRM) & 0x0F;
}


/**
 * [ADCDMA::ADCInit description]
 * @param ADC_Clk [description]
 */
bool ADCDMA::Init(uint32_t ADC_Clk)
{
  uint32_t pclkdiv, pclk;

  // Must disable the NVIC ADC interrupt for DMA use.
  NVIC_DisableIRQ(_LPC_ADC_IRQ);

  // Clear out the memory where the values are going
  memset(adc_buf, 0, sizeof(adc_buf));

  // Enable CLOCK for ADC controller
  LPC_SC->PCONP |= (1 << 12);


  // all the related pins are set to ADC inputs, AD0.0~7
  LPC_PINCON->PINSEL0 &= ~0x000000F0;   // P0.2~3, A0.6~7, function 10
  LPC_PINCON->PINSEL0 |= 0x000000A0;
  LPC_PINCON->PINSEL1 &= ~0x003FC000;   // P0.23~26, A0.0~3, function 01
  LPC_PINCON->PINSEL1 |= 0x00154000;
  LPC_PINCON->PINSEL3 |= 0xF0000000;    // P1.30~31, A0.4~5, function 11

  /* No pull-up no pull-down (function 10) on these ADC pins. */
  LPC_PINCON->PINMODE0 &= ~0x000000F0;
  LPC_PINCON->PINMODE0 |= 0x000000A0;
  LPC_PINCON->PINMODE1 &= ~0x003FC000;
  LPC_PINCON->PINMODE1 |= 0x002A8000;
  LPC_PINCON->PINMODE3 &= ~0xF0000000;
  LPC_PINCON->PINMODE3 |= 0xA0000000;

  /* By default, the PCLKSELx value is zero, thus, the PCLK for
  all the peripherals is 1/4 of the SystemFrequency. */
  /* Bit 24~25 is for ADC */
  pclkdiv = (LPC_SC->PCLKSEL0 >> 24) & 0x03;

  switch (pclkdiv) {
  case 0x00:
  default:
    pclk = SystemFrequency / 4;
    break;

  case 0x01:
    pclk = SystemFrequency;
    break;

  case 0x02:
    pclk = SystemFrequency / 2;
    break;

  case 0x03:
    pclk = SystemFrequency / 8;
    break;
  }

  _LPC_ADC_ID->ADCR = ( 0x01 << 0 ) |  // SEL=1,select channel 0~7 on ADC0
                      ( ( pclk  / ADC_Clk - 1 ) << 8 ) |  // CLKDIV = Fpclk / ADC_Clk - 1
                      ( 0 << 17 ) |       // CLKS = 0, 11 clocks/10 bits
                      ( 0 << 24 ) |       // START = 0 A/D conversion stops
                      ( 0 << 27 );        // EDGE = 0 (CAP/MAT singal falling,trigger A/D conversion)

  // Only allow the channels we want to read access to trigger the correct interrupt signal
  _LPC_ADC_ID->ADINTEN = ( 1 << 8 ) |
                         ( 1 << 7 ) |
                         ( 1 << 6 ) |
                         ( 1 << 5 ) |
                         ( 1 << 4 ) |
                         ( 1 << 3 ) |
                         ( 1 << 2 ) |
                         ( 1 << 1 ) |
                         ( 1 << 0 ) ;

  ADC_burst_on();
  ADC_start();
  isInit = true;
//   NVIC_EnableIRQ(_LPC_ADC_IRQ);

  return true;
}


/**
 * [ADCDMA::ADC_start Start all ADC operations.]
 */
void ADCDMA::ADC_start(void)
{
  // make sure it's in operational mode
  _LPC_ADC_ID->ADCR |= ( 1 << 21 );
}


/**
 * [ADCDMA::ADC_stop Stop all ADC operations.]
 */
void ADCDMA::ADC_stop(void)
{
  // place in power-down mode
  _LPC_ADC_ID->ADCR &= ~( 1 << 21 );
}


/**
 * [ADCDMA::ADC_burst_on Turn on BURST reads.]
 */
void ADCDMA::ADC_burst_on(void)
{
  if (burstEn == true)
    return;

  _LPC_ADC_ID->ADCR |= ( 1 << 16 );
  burstEn = true;
}


/**
 * [ADCDMA::ADC_burst_on Turn off BURST reads.]
 */
void ADCDMA::ADC_burst_off(void)
{
  if (burstEn == false)
    return;

  _LPC_ADC_ID->ADCR &= ~( 1 << 16 );
  burstEn = false;
}


/**
 * [ADCDMA::deselect_channels description]
 */
void ADCDMA::deselect_channels(void)
{
  _LPC_ADC_ID->ADCR &= ~(0xFF);
}


void ADCDMA::enable_channels(void)
{
  _LPC_ADC_ID->ADCR |= 0xFF;
}


/**
 * [ADCDMA::enable_channel description]
 * @param channel [description]
 */
void ADCDMA::enable_channel(uint8_t channel)
{
  _LPC_ADC_ID->ADCR |= (1 << 24) | (1 << channel);
}


/**
 * [ADCDMA::ADCRead description]
 * @param  channel [description]
 * @return            [description]
 */
uint32_t ADCDMA::Read(uint8_t channel)
{
  uint32_t regVal;

  // channel number is 0 through 7
  if (channel >= ADC_NUM_CHANNELS)
    channel = 0;     // reset channel number to 0

  deselect_channels();
  enable_channel(channel);

  do {
    // read result of A/D conversion
    regVal = ADC_GET_READING(_LPC_ADC_ID->ADDR0 + channel);
  } while (regVal & ADC_DONE);

  // _LPC_ADC_ID->ADCR &= 0xF8FFFFFF;    // stop ADC now
  ADC_stop();

  if (regVal & ADC_OVERRUN) // save data when it's not overrun, otherwise, return zero
    return 0xFFFFFFFF;

  return ADC_GET_READING(regVal);
}


/**
 * [ADCDMA::ADCBurstRead description]
 */
void ADCDMA::BurstRead(void)
{
  // Start bits need to be zero before BURST mode can be set.
  if (_LPC_ADC_ID->ADCR & (0x7 << 24))
    _LPC_ADC_ID->ADCR &= ~(0x7 << 24);

  deselect_channels();

  // Read all channels, 0 through 7.
  _LPC_ADC_ID->ADCR |= 0xFF;
  ADC_burst_on();
}
