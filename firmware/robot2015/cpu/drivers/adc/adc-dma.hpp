#pragma once

#include "robot.hpp"
#include "mbed.h"
#include "dma.hpp"


#define ADC_SAMPLE_RATE 10000  // 10kHz for reading multiplexed channels
#define ADC_SAMPLES_PER_CHAN 2
#define ADC_NUM_CHANNELS 3
#define _LPC_ADC_ID LPC_ADC
#define _LPC_ADC_IRQ ADC_IRQn

#define _ROBOT_ADC_PIN_0 RJ_BALL_DETECTOR
#define _ROBOT_ADC_PIN_1 RJ_BATT_SENSE
#define _ROBOT_ADC_PIN_2 RJ_5V_SENSE

// The ADC channel that will trigger the interrupt to begin all ADC channel readings
#define ADC_INT_TRIGGER_CHAN _ROBOT_ADC_PIN_0

#define ADC_GET_READING(x) (((uint32_t)(x) >> 0x04) & 0xFFF)

#define ADC_OFFSET          0x10
#define ADC_INDEX           4
#define ADC_DONE            0x80000000
#define ADC_OVERRUN         0x40000000
#define ADC_ADINT           0x00010000
#define ADC_CLK             1000000   // set to 1Mhz


/*
// Initialize ADC
ADCInit( ADC_CLK );

// ADC Interrupt is needed, but NVIC is not necessary for ADC DMA.
NVIC_DisableIRQ(ADC_IRQn);
DMA_Init();

// on DMA channel 0, source is ADC, Destination is Memory
// Enable channel and IE bit
DMAChannel_Init( 0, P2M );

// Set TC/Err int mask, channel enabled, dest. peripheral is memory(0x0). src peripheral is ADC(8), and transfer type is the P2M(2)
LPC_GPDMACH0->CConfig = 0xC001 | (DMA_ADC << 1) | (0x00 << 6) | (0x02 << 11);
ADCBurstRead();
while ( !ADCDMA0Done )
ADCDMA0Done = 0;
*/

void ADC_start(void);
void ADC_stop(void);
void ADC_burst_on(void);
void ADC_burst_off(void);


#ifndef _BV
#define _BV(_x_) (1UL << (_x_))
#endif


class ADCDMA
{
 public:
  ADCDMA(void);
  ~ADCDMA(void);

  bool Check(void);
  bool Poll(void);
  bool Init(uint32_t);
  uint32_t Read(uint8_t);
  void BurstRead(void);

  void ADC_start(void);
  void ADC_stop(void);
  void ADC_burst_on(void);
  void ADC_burst_off(void);
  uint8_t Offset(void);

 protected:
  bool InterruptTest(void);
  void enable_channel(uint8_t);
  void enable_channels(void);
  static uint8_t dmaChannelNum;
  uint32_t adc_buf[ADC_NUM_CHANNELS][ADC_SAMPLES_PER_CHAN];

 private:
  bool isInit;
  bool dmaTransferComplete[2];
  bool ADC_Interrupt_Done_Flag;
  bool ADC_int_done;
  bool burstEn;

  bool shutdown(void);
  void deselect_channels(void);
  void DMA_IRQHandler(void);
  void ADC_IRQHandler(void);

  volatile uint32_t ADCIntDone;
  volatile uint32_t BurstCounter;
  volatile uint32_t overrun_count;
  volatile uint32_t channel_flag;
};
