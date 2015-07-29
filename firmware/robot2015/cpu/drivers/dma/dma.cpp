#include "dma.hpp"

/*
volatile uint32_t DMATCCount = 0;
volatile uint32_t DMAErrCount = 0;
volatile uint32_t ADCDMA0Done = 0;
volatile uint32_t ADCDMA1Done = 0;
*/

/**
 *
 */
DMA::DMA(void)
{
  isInit = false;

  if (Init())
    LOG(INF1, "DMA setup successfully completed!");
}


/**
 *
 */
DMA::~DMA(void)
{
  isInit = false;
}


/**
 * [DMA::setSrc description]
 * @param addr [description]
 */
void DMA::setSrc(LPC_GPDMACH_TypeDef *dma_channel, uint32_t *addr)
{
  if (dma_channel == NULL)
    dma_channel = chan;

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
 * [DMA::find_channel description]
 * @return  [description]
 */
LPC_GPDMACH_TypeDef *DMA::find_channel(void)
{
  uint8_t enabled_channels = (_LPC_DMA_ID->DMACEnbldChns) & 0xFF;

  // Loop through all available channels until a good one is found
  for (int i = 0; i < DMA_NUM_CHANNELS; i++) {
    bool chan_good = ((enabled_channels >> i) & 0x01);

    if (chan_good)
      return (LPC_GPDMACH_TypeDef *)((uint32_t)_LPC_DMA_CHAN0 + (i * sizeof(LPC_GPDMACH_TypeDef *)));
  }

  return (LPC_GPDMACH_TypeDef *)0x00;
}


/**
 * [DMA::enableController description]
 */
uint8_t DMA::enable_controller(bool endianness)
{
  _LPC_DMA_ID->DMACConfig |= (0x01 | (endianness << 1));
  return _LPC_DMA_ID->DMACConfig & 0x03;
}


/**
 * [DMA::disable_controller description]
 * @return  [description]
 */
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
 * [DMA::reset_periph_modes description]
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


/**
 * [DMA::clear_int_flags description]
 */
void DMA::clear_int_flags(void)
{
  // doesn't work yet!
  _LPC_DMA_ID->DMACIntTCClear = ( 1 << 0 );
}



/**
 * [DMA::clear_err_flags description]
 */
void DMA::clear_err_flags(void)
{
  // doesn't work yet!
  _LPC_DMA_ID->DMACIntErrClr = ( 1 << 0 );
}


/**
 * [DMA_Init description]
 */
bool DMA::Init(void)
{
  NVIC_DisableIRQ(DMA_IRQn);
  clear_int_flags();
  clear_err_flags();

  chan = find_channel();
  LOG(OK, "DMA Channel Addr:\t%u", chan);

  if (chan == 0)
    return false;

  // Enable CLOCK into GPDMA controller
  LPC_SC->PCONP |= (1 << 29);
  // reset_periph_modes();
  // set_periph_mode(1);  // This sets it up for use with Timer0 match0 as being the selection

  // Sync enabled
  _LPC_DMA_ID->DMACSync = 0xFFFF;

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

  set_src_periph(chan, DMA_ADC_REQ_ID);
  // set_dst_periph(chan, DMA_ADC_REQ_ID);

  isInit = true;

  return true;
}


/**
 * [DMA::start description]
 */
void DMA::Start(void)
{
  if (isInit == false)
    return;

  // // Enable the channel once everything is properly configured
  enable_channel(chan);

  NVIC_EnableIRQ(DMA_IRQn);
}
