#pragma once

#include "robot.hpp"
#include "adc-dma.hpp"

#define _LPC_DMA_ID     LPC_GPDMA
#define _LPC_DMA_CHAN0  LPC_GPDMACH0
#define _LPC_DMA_CHAN1  LPC_GPDMACH1
#define _LPC_DMA_CHAN2  LPC_GPDMACH2
#define _LPC_DMA_CHAN3  LPC_GPDMACH3
#define _LPC_DMA_CHAN4  LPC_GPDMACH4
#define _LPC_DMA_CHAN5  LPC_GPDMACH5
#define _LPC_DMA_CHAN6  LPC_GPDMACH6
#define _LPC_DMA_CHAN7  LPC_GPDMACH7

#define DMA_NUM_CHANNELS    (0x08)

#define DMA_BURST_SIZE_1    (0x00)
#define DMA_BURST_SIZE_4    (0x01)
#define DMA_BURST_SIZE_8    (0x02)
#define DMA_BURST_SIZE_16   (0x03)
#define DMA_BURST_SIZE_32   (0x04)
#define DMA_BURST_SIZE_64   (0x05)
#define DMA_BURST_SIZE_128  (0x06)
#define DMA_BURST_SIZE_256  (0x07)

#define DMA_LITTLE_ENDIAN   (0x00)
#define DMA_BIG_ENDIAN      (0x01)

#define DMA_SSP0_TX_REQ_ID  (0x00)
#define DMA_SSP0_RX_REQ_ID  (0x01)
#define DMA_SSP1_TX_REQ_ID  (0x02)
#define DMA_SSP1_RX_REQ_ID  (0x03)
#define DMA_ADC_REQ_ID      (0x04)
#define DMA_I2S0_REQ_ID     (0x05)
#define DMA_I2S1__REQ_ID    (0x06)
#define DMA_DAC__REQ_ID     (0x07)

#define DMA_WIDTH_BYTE      (0x00)
#define DMA_WIDTH_HALFWORD  (0x01)
#define DMA_WIDTH_WORD      (0x02)

#define DMA_SET_SRC_SIZE(x)             (((x) << 12) & 0x07)
#define DMA_SET_DST_SIZE(x)             (DMA_SET_SRC_SIZE(x) << 0x03)
#define DMA_SET_SRC_WIDTH(x)            (((x) << 18) & 0x07)
#define DMA_SET_DST_WIDTH(x)            (DMA_SET_SRC_WIDTH(x) << 0x03)
#define DMA_SET_SRC_INCREMENT_MODE(x)   (((x) << 26) & 0x01)
#define DMA_SET_DST_INCREMENT_MODE(x)   (DMA_SET_SRC_INCREMENT_MODE(x) << 0x01)

#define DMA_ADDR_INCREMENT_NO   0x00
#define DMA_ADDR_INCREMENT_YES  0x01


class DMA
{
  public:
    DMA(void);
    ~DMA(void);

    void Init(void);
    void setSrc(LPC_GPDMACH_TypeDef *, uint32_t *);
    void setDst(LPC_GPDMACH_TypeDef *, uint32_t *);
    void start(void);

  protected:
    uint8_t enable_controller(bool = DMA_LITTLE_ENDIAN);
    uint8_t disable_controller(void);
    // uint32_t channel_init(uint32_t, uint32_t);
    LPC_GPDMACH_TypeDef *find_channel(void);

  private:
    bool isInit;

    void set_periph_mode(uint8_t);
    void set_src_periph(LPC_GPDMACH_TypeDef *, uint8_t);
    void set_dst_periph(LPC_GPDMACH_TypeDef *, uint8_t);
    void reset_periph_modes(void);
    void clear_error(uint8_t);
    void clear_errors(void);
    void enable_channel(LPC_GPDMACH_TypeDef *);
    void disable_channel(LPC_GPDMACH_TypeDef *);
    void disable_channels(void);
    LPC_GPDMACH_TypeDef *chan;
};
