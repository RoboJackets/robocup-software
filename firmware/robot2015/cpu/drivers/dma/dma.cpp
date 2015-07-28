#include "dma.hpp"

volatile uint32_t DMATCCount = 0;
volatile uint32_t DMAErrCount = 0;
volatile uint32_t ADCDMA0Done = 0;
volatile uint32_t ADCDMA1Done = 0;


/**
 * [DMA_IRQHandler description]
 */
/*
void DMA::DMA_IRQHandler(void)
{
  uint32_t regVal = _LPC_DMA_ID->IntTCStat;

  if (regVal) {
    DMATCCount++;
    _LPC_DMA_ID->IntTCClear = regVal;

    if ( regVal & 0x01 ) {
      ADCDMA0Done = 1;
    } else if ( regVal & 0x02 ) {
      ADCDMA1Done = 1;
    }

    ADC_burst_off();
  }

  regVal = _LPC_DMA_ID->IntErrStat;

  if (regVal) {
    DMAErrCount++;

    _LPC_DMA_ID->IntErrClr = regVal;
    ADC_burst_off();
  }
}
*/


DMA::DMA(void)
{
  isInit = false;
  Init();
  log(INF1, "DMA", "DMA setup complete.");
}


DMA::~DMA(void)
{
  isInit = false;
}

/**
 * [DMA::clear_error description]
 * @param channel [description]
 */
void DMA::clear_error(uint8_t channel)
{
  _LPC_DMA_ID->DMACIntErrClr = ( 1 << channel );
}


/**
 * [DMA::clear_errors description]
 */
void DMA::clear_errors(void)
{
  for (int i = 0; i < DMA_NUM_CHANNELS; i++)
    clear_error(i);
}


/**
 * [DMA::setSrc description]
 * @param addr [description]
 */
void DMA::setSrc(LPC_GPDMACH_TypeDef *dma_channel, uint32_t *addr)
{
  dma_channel->DMACCSrcAddr = (uint32_t)addr;
}


/**
 * [DMA::setDst description]
 * @param addr [description]
 */
void DMA::setDst(LPC_GPDMACH_TypeDef *dma_channel, uint32_t *addr)
{
  if (dma_channel == NULL)
    dma_channel = chan;

  dma_channel->DMACCDestAddr = (uint32_t)addr;
}


/**
 * [DMA::enableController description]
 */
uint8_t DMA::enable_controller(bool endianness)
{
  _LPC_DMA_ID->DMACConfig |= (0x01 | (endianness << 1));
  return _LPC_DMA_ID->DMACConfig & 0x03;
}

uint8_t DMA::disable_controller(void)
{
  // Check for any enabled channels
  if (_LPC_DMA_ID->DMACEnbldChns & 0xFF)
    disable_channels();

  // Shutdown the DMA controller
  _LPC_DMA_ID->DMACConfig &= ~(0x01);

  return _LPC_DMA_ID->DMACConfig & 0x03;
}


/**
 * [DMA::enable_channel description]
 * @param dma_channel [description]
 */
void DMA::enable_channel(LPC_GPDMACH_TypeDef *dma_channel)
{
  dma_channel->DMACCConfig |= 0x01;
}


/**
 * [DMA::enable_channel description]
 * @param dma_channel [description]
 */
void DMA::disable_channel(LPC_GPDMACH_TypeDef *dma_channel)
{
  dma_channel->DMACCConfig &= ~(0x01);
}


/**
 * [DMA::disable_channels description]
 */
void DMA::disable_channels(void)
{
  for (int i = 0; i < DMA_NUM_CHANNELS; i++)
    disable_channel(_LPC_DMA_CHAN0 + i);
}


/**
 * [DMA::set_periph_mode description]
 * @param bit [description]
 */
void DMA::set_periph_mode(uint8_t mode)
{
  LPC_SC->DMAREQSEL |= (1 << mode);
}


/**
 * [DMA::reset_periph description]
 */
void DMA::reset_periph_modes(void)
{
  LPC_SC->DMAREQSEL = 0x00;
}


/**
 * [DMA::set_src_periph description]
 */
void DMA::set_src_periph(LPC_GPDMACH_TypeDef *dma_channel, uint8_t mode)
{
  dma_channel->DMACCConfig |= ((mode << 1) & 0x0F);
}


/**
 * [DMA::set_dst_periph description]
 */
void DMA::set_dst_periph(LPC_GPDMACH_TypeDef *dma_channel, uint8_t mode)
{
  dma_channel->DMACCConfig |= ((mode << 6) & 0x0F);
}


LPC_GPDMACH_TypeDef *DMA::find_channel(void)
{
  uint8_t enabled_channels = (_LPC_DMA_ID->DMACEnbldChns) & 0xFF;

  for (int i = 0; i < DMA_NUM_CHANNELS; i++) {
    uint8_t chan_good = ((enabled_channels >> i) & 0x01);

    if (chan_good)
      return (LPC_GPDMACH_TypeDef *)(_LPC_DMA_CHAN0 + (i * sizeof(LPC_GPDMACH_TypeDef *)));
  }

  return (LPC_GPDMACH_TypeDef *)0x00;
}


/**
 * [DMA_Init description]
 */
void DMA::Init(void)
{
  NVIC_DisableIRQ(ADC_IRQn);
  NVIC_DisableIRQ(DMA_IRQn);

  //_LPC_DMA_ID->DMACIntTCClear = ( 1 << 0 );
  //_LPC_DMA_ID->DMACIntErrClr = ( 1 << 0 );

  chan = find_channel();

  if (chan == 0)
    return;

  /* Enable CLOCK into GPDMA controller */
  LPC_SC->PCONP |= (1 << 29);
  //reset_periph_modes();
  //set_periph_mode(1);  // This sets it up for use with Timer0 match0 as being the selection

  /* sync enabled */
  _LPC_DMA_ID->DMACSync = 0xFFFF;
  //_LPC_DMA_ID->DMACIntTCClear = 0x03;
  //_LPC_DMA_ID->DMACIntErrClr = 0x03;

  // Startup the global DMA controller interface
  while (enable_controller(DMA_LITTLE_ENDIAN) < 0x01) { /* wait */ }

  // Setup everything, but don't start any transfers
  chan->DMACCControl = (
                         0x01  // Tranfer size - defined in the units set here.
                         | DMA_SET_SRC_SIZE(DMA_BURST_SIZE_1)
                         | DMA_SET_DST_SIZE(DMA_BURST_SIZE_1)
                         | DMA_SET_SRC_WIDTH(DMA_WIDTH_WORD)
                         | DMA_SET_DST_WIDTH(DMA_WIDTH_WORD)
                         | DMA_SET_SRC_INCREMENT_MODE(DMA_ADDR_INCREMENT_NO)
                         | DMA_SET_DST_INCREMENT_MODE(DMA_ADDR_INCREMENT_NO)
                       );

  // setSrc();
  // setDst();

  set_src_periph(chan, DMA_ADC_REQ_ID);
  // set_dst_periph(chan, DMA_ADC_REQ_ID);

  isInit = true;
}


/**
 * [DMA::start description]
 */
void DMA::start()
{
  if (isInit == false)
    return;

  // // Enable the channel once everything is properly configured
  enable_channel(chan);

  NVIC_EnableIRQ(DMA_IRQn);
}


/*
bool DMA::channel_init(uint32_t channel, uint32_t DMAMode)
{
  if (channel == 0) {
    dmaTransferComplete[0] = false;

    // Clear DMA channel 0 terminal count.
    _LPC_DMA_ID->IntTCClear = ( 1 << 0 );
    _LPC_DMA_ID->IntErrClr = ( 1 << 0 );
    _LPC_DMA_CHAN0->CControl = 0;
    _LPC_DMA_CHAN0->CConfig = 0;

    if (DMAMode == P2M) {

      // Ch0 set for P2M transfer from ADC to mempry.
      //setSrc(_LPC_DMA_CHAN0, );
      //setDst(_LPC_DMA_CHAN0, );

      // The burst size is set to 8, source and dest transfer width is
      // 32 bits(word), src addr increment by 1. dst addr increment by 1. Terminal
      // Count Int enable
      _LPC_DMA_CHAN0->CControl = (DMA_SIZE & 0x0FFF)
                                 | (0x00 << 12)
                                 | (0x00 << 15)
                                 | (0x02 << 18)
                                 | (0x02 << 21)
                                 | (1 << 26)
                                 | (1 << 27)
                                 | 0x80000000;

      return true;
    } else {

      return false;
    }
  } else if (channel == 1) {

    dmaTransferComplete[1] = false;

    // Clear DMA channel 1 terminal count.
    _LPC_DMA_ID->IntTCClear = ( 1 << 1 );
    _LPC_DMA_ID->IntErrClr = ( 1 << 1 );
    _LPC_DMA_CHAN1->CControl = 0;
    _LPC_DMA_CHAN1->CConfig = 0;

    if (DMAMode == P2M) {

      // Ch1 set for P2M transfer from ADC to mempry.
      _LPC_DMA_CHAN1->CSrcAddr = DMA_SRC;
      _LPC_DMA_CHAN1->CDestAddr = DMA_DST;

      // The burst size is set to 8, source and dest transfer width is
      // bits(word), src addr increment by 1. dst addr unchange. Terminal
      Count Int enable
      _LPC_DMA_CHAN1->CControl = (DMA_SIZE & 0x0FFF)
                                 | (0x00 << 12)
                                 | (0x00 << 15)
                                 | (0x02 << 18)
                                 | (0x02 << 21)
                                 | (1 << 26)
                                 | (1 << 27)
                                 | 0x80000000;

      return true;
    } else {

      return false;
    }
  }

  return false;
}
*/
