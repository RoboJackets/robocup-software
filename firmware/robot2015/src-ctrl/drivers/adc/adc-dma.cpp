#include "adc-dma.hpp"

#include <cstdarg>

#include "pinmap.h"
#include "logger.hpp"

// ADC pin mapping for the LPC1768 microcontroller based mbed

static const PinMap RJ_PinMap_ADC[] = {{P0_23, ADC0_0, 1},
                                       {P0_24, ADC0_1, 1},
                                       {P0_25, ADC0_2, 1},
                                       {P0_26, ADC0_3, 1},
                                       {P1_30, ADC0_4, 3},
                                       {P1_31, ADC0_5, 3},
                                       {P0_2, ADC0_7, 2},
                                       {P0_3, ADC0_6, 2},
                                       {NC, NC, 0}};

uint8_t ADCDMA::dmaChannelNum = 0;
bool ADCDMA::burstEn = false;

/**
 *
 */
ADCDMA::ADCDMA() {
    isInit = false;
    ADC_int_done = true;

    BurstCounter = 0;
    overrun_count = 0;
    channel_flag = 0;
    ADCIntDone = 0;

    // Clear out a section of memory for storing the ADC readings
    memset(adc_buf, 0, sizeof(adc_buf));
    memset(dmaTransferComplete, 0, sizeof(dmaTransferComplete));
}

/**
 *
 */
ADCDMA::~ADCDMA() {
    if (ADC_powerdown())
        LOG(INF3, "ADC DMA killed. All ADC interrupts removed.");
}

/**
 * [ADCDMA::Init Setup the FPGA interface]
 * @return  [The initialization error code.]
 */

// ERR_t ADCDMA::Init()
//{
//  if (isInit)
// return 0;

// ADCStruct.adcRate = ADC_SAMPLE_RATE;
// ADCStruct.burstMode = Burst_Mode_Flag;
// ADCStruct.bitsAccuracy = ADC_10BITS;

// Chip_ADC_Init(_LPC_ADC_ID, &ADCStruct);

// Enable all of the ADC channels
// for (int i = 0; i < ADC_NUM_CHANNELS_CHANNELS; i++)
// Chip_ADC_Channel_Int_Cmd(_LPC_ADC_ID, ADC_INT_TRIGGER_CHAN, ENABLE);

// dmaChannelNum = Chip_DMA_GetFreeChannel(LPC_GPDMA, _GPDMA_CONN_ADC);
// LOG(INF2, "ADC setup complete. Now linking requests to DMA channel %u.",
// dmaChannelNum);

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
// LOG(INF1, "ADC using DMA setup complete.");

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
 * @return  [Returns TRUE when the DMA transfer was complete & the function
 * retreived the values.]
 */
bool ADCDMA::Poll() {
    // When transfer complete do this block.
    if (dmaTransferComplete[0]) {
        dmaTransferComplete[0] = false;

        // Read all ADCs
        for (int i = 0; i < ADC_NUM_CHANNELS; i++)
            LOG(INF2, "Polled reading value:\t%u",
                ADC_GET_READING(adc_buf[i][0]));

        for (int i = 0; i < ADC_NUM_CHANNELS; i++) {
            // return TRUE is ANY channel reading is obtained
            if (adc_buf[i][0] != 0) return true;
        }
    }

    return false;
}

/**
 * [ADCDMA::InterruptTest description]
 */
bool ADCDMA::InterruptTest() {
    /* Enable ADC Interrupt */
    NVIC_EnableIRQ(_LPC_ADC_IRQ);
    // Chip_ADC_Channel_Int_Cmd(_LPC_ADC_ID, ADC_INT_TRIGGER_CHAN, ENABLE);

    /* Enable burst mode if any, the AD converter does repeated conversions
       at the rate selected by the CLKS field in burst mode automatically */
    ADC_burst_on();

    bool Interrupt_Continue_Flag = true;
    ADC_int_done = true;

    while (Interrupt_Continue_Flag) {
        if (!burstEn && ADC_int_done) ADC_int_done = false;
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
/*
void ADCDMA::DMA_IRQHandler()
{
  // if (Chip_DMA_Interrupt(LPC_GPDMA, ADCDMA::dmaChannelNum) == SUCCESS) {
  // channelTC++;
  // } else {
  // Process error here

  // turn off burst and disable
  if (0) {
    ADC_burst_off();
    ADC_stop();
  }

  LOG(SEVERE, "Unable to successfully configure the DMA for use with the ADC
pins.");
  // }
}
*/

/**
 * [ADC_IRQHandler description]
 */
/*
extern "C" void ADC_IRQHandler()
{
  // uint32_t dummy;
  uint32_t regVal;

  // Reading the ADC will clear the interrupt
  regVal = ((_LPC_ADC_ID->ADSTAT) >> 8) & 0xFF;

  // check OVERRUN error first
  if (regVal) {

  }

    overrun_count++;

    for (int i = 0; i < ADC_NUM_CHANNELS; i++) {
      uint8_t chan_overrun = (regVal >> i);

       //if overrun, just read ADDR to clear
  // regVal variable has been reused.
  //if (chan_overrun)
  uint32_t dummy = _LPC_ADC_ID->ADDR0 + i;
  }

  ADC_stop();
  ADC_burst_off();
  ADC_int_done = 1;
  }

  for (int i = 0; i < ADC_NUM_CHANNELS; i++)
  if (regVal &_BV(i))
    adc_buf[i][0] = ADC_GET_READING(_LPC_ADC_ID->ADDR0 + i);


  // BurstCounter++;
  channel_flag |= (regVal & 0xFF);

  if ((channel_flag & 0xFF) == 0xFF)
  {
  ADC_burst_off();
  channel_flag = 0;
  ADCIntDone = 1;
  }

}
*/

/**
 * [ADCDMA::Offset Retrieve the ADC offset value set by the bootloader at
 * startup.]
 * @return  [ADC offset value]
 */
uint8_t ADCDMA::Offset() { return ADC_GET_READING(_LPC_ADC_ID->ADTRM) & 0x0F; }

/**
 * [ADCDMA::AddChannel description]
 * @param pin [description]
 */
void ADCDMA::SetChannels(std::initializer_list<PinName> pins) {
    std::vector<PinName> pinsList(pins);
    std::vector<ADCPin_t>::iterator it;

    for (it = adc_chan.begin(); it < adc_chan.end(); it++) {
        adc_chan.insert(it, adc_chan.begin(), adc_chan.end());

        it->pin_name = pinsList.at(adc_chan.size());
        // obj.pin_obj->adc = (ADCName)pinmap_peripheral(obj.pin_name,
        // RJ_PinMap_ADC);
    }

    LOG(INIT, "ADC Channels Vec Size: %u\r\n", adc_chan.size());
}

bool ADCDMA::Start() { return init_channels(); }

/**
 * [ADCDMA::init_channels description]
 * @return  [description]
 */
bool ADCDMA::init_channels() {
    // return FALSE if no channels (pins) have been added yet
    if (adc_chan.empty() || (isInit == false)) return false;

    /*
      // THIS BREAKS THINGS!!
      // Check for valid ADC pins
      for (std::vector<PinName>::iterator pin = adc_chan.begin(); pin !=
      adc_chan.end(); ++pin) {

        // MBED_ASSERT(obj->adc != (ADCName)NC);
      }
    */

    // Clear out the memory where the values are going
    // memset(adc_buf, 0, sizeof(adc_buf));

    // Must disable the NVIC ADC interrupt for DMA use.
    NVIC_DisableIRQ(_LPC_ADC_IRQ);

    // ensure power is turned on for the ADC
    LPC_SC->PCONP |= (1 << 12);

    // set PCLK of ADC to /1
    LPC_SC->PCLKSEL0 &= ~(0x3 << 24);
    LPC_SC->PCLKSEL0 |= (0x1 << 24);
    uint32_t PCLK = SystemCoreClock;

    // calculate minimum clock divider
    uint32_t MAX_ADC_CLK = 13000000;
    uint32_t clkdiv = div_round_up(PCLK, MAX_ADC_CLK) - 1;

    // Set the generic software-controlled ADC settings
    _LPC_ADC_ID->ADCR =
        (0 << 0)  // SEL: 0 = no channels selected initially
        | (clkdiv
           << 8)  // CLKDIV: PCLK max ~= 25MHz, /25 to give safe 1MHz at fastest
        |
        (0 << 17);  // CLKS: not applicable
    //    | (0 << 24)     // START: 0 = no start
    //   | (0 << 27);    // EDGE: not applicable

    // ADC_burst_on();
    ADC_start();

    // Configure the pins for reading voltages (ie. no pull-ups/pull-downs)
    for (std::vector<ADCPin_t>::iterator pin = adc_chan.begin();
         pin != adc_chan.end(); ++pin)
        pinmap_pinout(pin->pin_name, RJ_PinMap_ADC);

    // Only allow the channels we want to read access to trigger the correct
    // interrupt signal
    _LPC_ADC_ID->ADINTEN = (1 << 8) |  // When set to 0, only the individual ADC
                                       // channels enabled here will generate an
                                       // interrupt
                           (0 << 7) |  // ADC0.7 - no mbed breakout pin (P0.2)
                           (0 << 6) |  // ADC0.6 - no mbed breakout pin (P0.3)
                           (0 << 5) |  // ADC0.5 - mbed p20
                           (0 << 4) |  // ADC0.4 - mbed p19
                           (0 << 3) |  // ADC0.3 - mbed p18
                           (1 << 2) |  // ADC0.2 - mbed p17
                           (1 << 1) |  // ADC0.1 - mbed p16
                           (1 << 0);   // ADC0.0 - mbed p15

    isInit = true;
    LOG(INF2, "ADC setup (using DMA) complete.");

    return true;
}

/**
 * [ADCDMA::ADC_start Start all ADC operations.]
 */
void ADCDMA::ADC_start() {
    // make sure it's in operational mode
    _LPC_ADC_ID->ADCR |= (1 << 21);
}

/**
 * [ADCDMA::ADC_stop description]
 */
void ADCDMA::ADC_stop() {
    // Stop conversion
    LPC_ADC->ADCR &= ~(1 << 24);
}

/**
 * [ADCDMA::ADC_burst_on Turn on BURST reads.]
 */
void ADCDMA::ADC_burst_on() {
    if (burstEn == true) return;

    _LPC_ADC_ID->ADCR |= (1 << 16);
    burstEn = true;
}

/**
 * [ADCDMA::ADC_burst_on Turn off BURST reads.]
 */
void ADCDMA::ADC_burst_off() {
    if (burstEn == false) return;

    _LPC_ADC_ID->ADCR &= ~(1 << 16);
    burstEn = false;
}

/**
 * [ADCDMA::enable_channel description]
 * @param channel [description]
 */
void ADCDMA::enable_channel(uint8_t channel) {
    _LPC_ADC_ID->ADCR |= (1 << 24) | (1 << channel);
}

void ADCDMA::enable_channels() { _LPC_ADC_ID->ADCR |= 0xFF; }

/**
 * [ADCDMA::deselect_channels description]
 */
void ADCDMA::deselect_channels() { _LPC_ADC_ID->ADCR &= ~(0xFF); }

/**
 * [ADCDMA::ADC_stop Stop all ADC operations.]
 */
bool ADCDMA::ADC_powerdown() {
    // place in power-down mode
    _LPC_ADC_ID->ADCR &= ~(1 << 21);
    ADC_burst_off();

    NVIC_DisableIRQ(_LPC_ADC_IRQ);
    isInit = false;

    return true;
}

/**
 * [ADCDMA::ADCRead description]
 * @param  channel [description]
 * @return            [description]
 */
uint32_t ADCDMA::Read(uint8_t channel) {
    if (isInit == false) return 0xFFFFFFFF;

    uint32_t data;

    // channel number is 0 through 7
    if (channel > adc_chan.size()) return 0xEEEEEEEE;

    deselect_channels();
    enable_channel(channel);

    do {
        // read result of A/D conversion
        data = ADC_GET_READING(_LPC_ADC_ID->ADGDR);
    } while (data & ADC_DONE);

    ADC_stop();

    LOG(INF2, "ADC reading from channel %u: %u", channel, data);

    if (data &
        ADC_OVERRUN)  // save data when it's not overrun, otherwise, return zero
        return 0xDDDDDDDD;

    return data;
}

/**
 * [ADCDMA::ADCBurstRead description]
 */
void ADCDMA::BurstRead() {
    if (isInit == false) return;

    // Start bits need to be zero before BURST mode can be set.
    _LPC_ADC_ID->ADCR &= ~(0x07 << 24);

    // Read only channels we care about, turn others off.
    deselect_channels();
    _LPC_ADC_ID->ADCR |= 0x03;

    ADC_burst_on();
}
