#ifdef COMPILE_WITH_DMA

#include "adc-dma.hpp"

ADCDMA::ADCDMA(void)
{
    isInit = false;
    dmaTransferComplete = false;
    // do stuff
    isInit = true;
}


ADCDMA::~ADCDMA(void)
{
    isInit = false;
}


ERR_t ADCDMA::Init(void)
{
    // Create a buffer to hold the ADC samples and clear it.
    // Note, we are going to sample two ADC inputs so they
    // end up in this buffer "interleaved". So you will want
    // a buffer twice this size to a real life given sample
    // frequency. See the printf() output for details.
    memset(adc_buf, 0, sizeof(adc_buf));

    // We use the ADC irq to trigger DMA and the manual says
    // that in this case the NVIC for ADC must be disabled.
    NVIC_DisableIRQ(ADC_IRQn);
    log(INF3, "ADC DMA INIT", "NVIC for ADC disabled.");

    // Power up the ADC and set PCLK
    LPC_SC->PCONP    |=  (1UL << 12);
    LPC_SC->PCLKSEL0 &= ~(3UL << 24); // PCLK = CCLK/4 96M/4 = 24MHz

    // Enable the ADC, 12MHz,  ADC0.0 & .1
    LPC_ADC->ADCR  = (1UL << 21) | (1UL << 8) | (3UL << 0);

    // FIX ME - put the correct pins ==========
    // Set the pin functions to ADC
    LPC_PINCON->PINSEL1 &= ~(3UL << 14);  /* P0.23, Mbed p15. */
    LPC_PINCON->PINSEL1 |=  (1UL << 14);
    LPC_PINCON->PINSEL1 &= ~(3UL << 16);  /* P0.24, Mbed p16. */
    LPC_PINCON->PINSEL1 |=  (1UL << 16);

    // Prepare an ADC configuration.
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
    ->attach_tc     ( &adc_TC0_callback )
    ->attach_err    ( &adc_ERR0_callback )
    ; // end conf.

    // Prepare configuration.
    dma.Setup( conf );
    log(INF2, "ADC DMA INIT", "DMA setup complete.");

    // Enable configuration.
    dma.Enable( conf );
    log(INF1, "ADC DMA INIT", "DMA enabled for ADC readings.");

    // Enable ADC irq flag (to DMA).
    // Note, don't set the individual flags,
    // just set the global flag.
    LPC_ADC->ADINTEN = 0x100;

    // Enable burst mode on inputs 0 and 1.
    LPC_ADC->ADCR |= (1UL << 16);

    return 0;
}


bool ADCDMA::Check(void)
{
    // When transfer complete do this block.
    if (dmaTransferComplete) {
        delete conf; // No memory leaks, delete the configuration.

        dmaTransferComplete = false;

        for (int i = 0; i < SAMPLE_BUFFER_LENGTH; i++) {
            int channel = (adc_buf[i] >> 24) & 0x7;
            int iVal = (adc_buf[i] >> 4) & 0xFFF;
            double fVal = 3.3 * (double)((double)iVal) / ((double)0x1000); // scale to 0v to 3.3v
            // pc.printf("Array index %02d : ADC input channel %d = 0x%03x %01.3f volts\n", i, channel, iVal, fVal);
        }

        return true;
    }

    return false;
}


// Configuration callback on TC
void ADCDMA::TC0_callback(void)
{
    MODDMA_Config *config = dma.getConfig();

    // Disbale burst mode and switch off the IRQ flag.
    LPC_ADC->ADCR &= ~(1UL << 16);
    LPC_ADC->ADINTEN = 0;

    // Finish the DMA cycle by shutting down the channel.
    dma.haltAndWaitChannelComplete( (MODDMA::CHANNELS)config->channelNum());
    dma.Disable( (MODDMA::CHANNELS)config->channelNum() );

    // Tell main() while(1) loop to print the results.
    dmaTransferComplete = true;

    // Clear DMA IRQ flags.
    if (dma.irqType() == MODDMA::TcIrq)
        dma.clearTcIrq();

    if (dma.irqType() == MODDMA::ErrIrq)
        dma.clearErrIrq();
}


// Configuration callback on Error
void ADCDMA::ERR0_callback(void)
{
    // Switch off burst conversions.
    LPC_ADC->ADCR |= ~(1UL << 16);
    LPC_ADC->ADINTEN = 0;

    log(SEVERE, "ADC DMA", "Unable to successfully configure the DMA for use with the ADC pins.");
}

#endif
