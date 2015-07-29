#ifdef COMPILE_WITH_DMA

#include "speaker-dma.hpp"


/**
 *
 */
Speaker::Speaker(void)
{
    isInit = false;
    // speaker_buf[SAMPLES_PER_READ][BUFFER_SIZE] = 0;
    // do stuff
    isInit = true;
}


/**
 *
 */
Speaker::~Speaker(void)
{
    isInit = false;
}


/**
 * [Speaker::Init Setup the FPGA interface]
 * @return  [The initialization error code.]
 */
ERR_t Speaker::Init(void)
{
    volatile int life_counter = 0;

    if (isInit)
        return 1;

    // Create a sinewave buffer for testing.
    for (int i =   0; i <=  90; i++)
        buf[0][i] =  (512 * sin(3.14159 / 180.0 * i)) + 512;

    for (int i =  91; i <= 180; i++)
        buf[0][i] =  buf[0][180 - i];

    for (int i = 181; i <= 270; i++)
        buf[0][i] =  512 - (buf[0][i - 180] - 512);

    for (int i = 271; i <  360; i++)
        buf[0][i] =  512 - (buf[0][360 - i] - 512);

    // Adjust the sinewave buffer for use with DAC hardware.
    for (int i = 0; i < BUFFER_SIZE; i++) {
        buf[0][i] = DAC_POWER_MODE | ((buf[0][i] << 6) & 0xFFC0);
        buf[1][i] = buf[0][i]; // Just create a copy of buffer0 to continue sinewave.
    }

    // Prepare the GPDMA system for buf[0].
    conf0 = new MODDMA_Config;
    conf0
    ->channelNum    ( MODDMA::Channel_0 )
    ->srcMemAddr    ( (uint32_t) &buf[0] )
    ->dstMemAddr    ( MODDMA::DAC )
    ->transferSize  ( BUFFER_SIZE )
    ->transferType  ( MODDMA::m2p )
    ->dstConn       ( MODDMA::DAC )
    ->attach_tc     ( &TC0_callback )
    ->attach_err    ( &ERR0_callback )
    ; // conf0 end

    // Prepare the GPDMA system for buf[1].
    conf1 = new MODDMA_Config;
    conf1
    ->channelNum    ( MODDMA::Channel_1 )
    ->srcMemAddr    ( (uint32_t) &buf[1] )
    ->dstMemAddr    ( MODDMA::DAC )
    ->transferSize  ( BUFFER_SIZE )
    ->transferType  ( MODDMA::m2p )
    ->dstConn       ( MODDMA::DAC )
    ->attach_tc     ( &TC1_callback )
    ->attach_err    ( &ERR1_callback )
    ; // conf1 end

    // Calculating the transfer frequency:
    // By default, the Mbed library sets the PCLK_DAC clock value
    // to 24MHz. One complete sinewave cycle in each buffer is 360
    // points long. So, for a 1Hz wave we would need to transfer 360
    // values per second. That would be 24000000/360 which is approx
    // 66,666. But that's no good! The count val is only 16bits in size
    // so bare this in mind. If you need to go slower you will need to
    // alter PCLK_DAC from CCLK/4 to CCLK/8.
    // For our demo we are going to have the sinewave run at 1kHz.
    // That's 24000000/360000 which is approx 66. Experimentation
    // however showed 65 to get closer to 1kHz (on my Mbed and scope
    // at least).
    LPC_DAC->DACCNTVAL = 65; // 6500 for 10Hz

    // Prepare first configuration.
    if (!dma.Prepare( conf0 )) {
        dma_err();
        return 0xFFFFFFFF;
    }

    // Begin (enable DMA and counter). Note, don't enable
    // DBLBUF_ENA as we are using DMA double buffering.
    LPC_DAC->DACCTRL |= (3UL << 2);

    isInit = true;
    return 0;
}


/**
 * [Speaker::TC0_callback description]
 */
void Speaker::TC0_callback(void)
{
    // Get configuration pointer.
    MODDMA_Config *config = dma.getConfig();

    // Finish the DMA cycle by shutting down the channel.
    dma.Disable( (MODDMA::CHANNELS)config->channelNum() );

    // Swap to buffer1
    dma.Prepare( conf1 );

    // Clear DMA IRQ flags.
    if (dma.irqType() == MODDMA::TcIrq)
        dma.clearTcIrq();
}


/**
 * [Speaker::TC1_callback description]
 */
void Speaker::TC1_callback(void)
{
    // Get configuration pointer.
    MODDMA_Config *config = dma.getConfig();

    // Finish the DMA cycle by shutting down the channel.
    dma.Disable( (MODDMA::CHANNELS)config->channelNum() );

    // Swap to buffer0
    dma.Prepare( conf0 );

    // Clear DMA IRQ flags.
    if (dma.irqType() == MODDMA::TcIrq)
        dma.clearTcIrq();
}


/**
 * [Speaker::ERR0_callback description]
 */
void Speaker::ERR0_callback(void)
{
    dma_err(void);
}


// Configuration callback on Error
void Speaker::ERR1_callback(void)
{
    dma_err(void);
}


void Speaker::dma_err(void)
{
    //error("Oh no! My Mbed EXPLODED! :( Only kidding, go find the problem");
    LOG(SEVERE, "Unable to successfully configure the DMA for use with the speaker pin's output.");
}

#endif  // COMPILE_WITH_DMA
