#pragma once

#include "mbed.h"

#include <vector>

#include "dma.hpp"


// Defines
#define ADC_SAMPLE_RATE       (10000)  // 10kHz for reading multiplexed channels
#define ADC_SAMPLES_PER_CHAN  (2)
#define ADC_NUM_CHANNELS      (3)
#define _LPC_ADC_ID           LPC_ADC
#define _LPC_ADC_IRQ          ADC_IRQn
#define _ROBOT_ADC_PIN_0      RJ_BALL_DETECTOR
#define _ROBOT_ADC_PIN_1      RJ_BATT_SENSE
#define _ROBOT_ADC_PIN_2      RJ_5V_SENSE
#define ADC_INT_TRIGGER_CHAN  _ROBOT_ADC_PIN_0
#define ADC_OFFSET            (0x10)
#define ADC_INDEX             (0x04)
#define ADC_DONE              (0x80000000)
#define ADC_OVERRUN           (0x40000000)
#define ADC_ADINT             (0x00010000)
#define ADC_CLK               (1000000)

#define ANALOGIN_MEDIAN_FILTER      1

#define ADC_10BIT_RANGE             0x3FF
#define ADC_12BIT_RANGE             0xFFF
#define ADC_RANGE                   ADC_12BIT_RANGE


// Macros
#ifndef ADC_GET_READING
#define ADC_GET_READING(x) (((uint32_t)(x) >> 0x04) & 0xFFF)
#endif

#ifndef _BV
#define _BV(_x_) (1UL << (_x_))
#endif


static inline int div_round_up(int x, int y)
{
  return (x + (y - 1)) / y;
}


typedef struct ADCPin {
  PinName pin_name;
  analogin_t* pin_obj;
} ADCPin_t;


class ADCDMA
{
 public:
  ADCDMA();
  ~ADCDMA();

  bool Start();
  void SetChannels(std::initializer_list<PinName> pins);
  bool Poll();
  uint32_t Read(uint8_t);
  void BurstRead();
  uint8_t Offset();
  static void ADC_burst_on();
  static void ADC_burst_off();

  static bool burstEn;

 protected:
  bool InterruptTest();
  void enable_channel(uint8_t);
  void enable_channels();
  void deselect_channels();
  bool init_channels();


  static uint8_t dmaChannelNum;
  uint32_t adc_buf[ADC_NUM_CHANNELS][ADC_SAMPLES_PER_CHAN];
  analogin_t _adc;

 private:
  bool isInit;
  bool dmaTransferComplete[2];
  bool ADC_Interrupt_Done_Flag;
  bool ADC_int_done;

  std::vector<ADCPin_t> adc_chan;

  volatile uint32_t ADCIntDone;
  volatile uint32_t BurstCounter;
  volatile uint32_t overrun_count;
  volatile uint32_t channel_flag;

  void ADC_start();
  void ADC_stop();
  bool ADC_powerdown();
  void ADC_IRQHandler();


};
