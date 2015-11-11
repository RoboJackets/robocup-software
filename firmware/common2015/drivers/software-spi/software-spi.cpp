#include <mbed.h>

#include "software-spi.hpp"

#include "logger.hpp"

// #include "pinmap.h"
// #include "timer-api.hpp"

// #define SW_SPI_TIMER            (LPC_TIM2)
// #define SW_SPI_TIMER_IRQn       (TIMER2_IRQn)

// namespace
// {
// const uint8_t timer_pconp_bit_num = 22;
// const uint8_t mosi_channel = 2;
// const uint8_t sck_channel = 1;
// const int match_count = 8;

// bool isInit = false;

// void software_spi_irq_handler()
// {
//     NVIC_DisableIRQ(SW_SPI_TIMER_IRQn);

//     timer_ClearMatch(SW_SPI_TIMER, mosi_channel);
//     timer_ClearMatch(SW_SPI_TIMER, sck_channel);

//     NVIC_ClearPendingIRQ(SW_SPI_TIMER_IRQn);
//     NVIC_EnableIRQ(SW_SPI_TIMER_IRQn);
// }

// void setup_timer(LPC_TIM_TypeDef* timer, uint8_t match_num, bool enable_int = false)
// {

//     NVIC_DisableIRQ(SW_SPI_TIMER_IRQn);

//     // enable power for the timer
//     LPC_SC->PCONP |= 1 << timer_pconp_bit_num;
//     LOG(INF1, "Power enabled for PCONP[%u]", timer_pconp_bit_num);

//     // disable the timer and reset its counter for initilization
//     timer_Disable(timer);
//     timer_Reset(timer);

//     // default to 1MHz (1 us ticks)
//     uint32_t prescale_count = (SystemCoreClock / 4) / 8000000;
//     timer_PrescaleSet(timer, prescale_count);
//     LOG(INF1, "Prescale count set to %u for channel %u", prescale_count, match_num);

//     // set the count for when to trigger the interrupt
//     timer_SetMatch(timer, match_num, match_count);
//     LOG(INF1, "Match count set to %u for channel %u", match_count, match_num);

//     // reset the counter when (counter == cnt)
//     timer_ResetOnMatchEnable(timer, match_num);
//     LOG(INF1, "Counter will RESET for channel %u", match_num);

//     timer_MatchDisableInt(timer, match_num);
//     LOG(INF1, "Interrupt DISABLED for channel %u", match_num);

//     // set to toggle the pin state on a match count - initial state of 0
//     timer_ExtMatchControlSet(timer, 0, TIMER_EXTMATCH_TOGGLE, match_num);
//     LOG(INF1, "Pin will TOGGLE for channel %u match", match_num);

// LOG(INIT,
//     "IRQ Addr:\t0x%08X\t(before remap)\r\n"
//     "  IRQ Priority:\t%10u\t(out of %u)\r\n"
//     "  IRQ Number:\t%10u",
//     NVIC_GetVector(SW_SPI_TIMER_IRQn),
//     NVIC_GetPriority(SW_SPI_TIMER_IRQn),
//     (1 << __NVIC_PRIO_BITS),
//     SW_SPI_TIMER_IRQn
//    );

// // set the location of the interrupt callback
// NVIC_SetVector(SW_SPI_TIMER_IRQn, (uint32_t)software_spi_irq_handler);
// NVIC_EnableIRQ(SW_SPI_TIMER_IRQn);

// LOG(INIT,
//     "IRQ Addr:\t0x%08X\t(after remap)\r\n"
//     "  Expected:\t0x%08X",
//     NVIC_GetVector(SW_SPI_TIMER_IRQn),
//     software_spi_irq_handler
//    );
// }
// }   // anonymous namespace

SoftwareSPI::SoftwareSPI(PinName mosi_pin, PinName miso_pin, PinName sck_pin, int bit_width)
{
    mosi = new DigitalOut(mosi_pin, 0);
    miso = new DigitalIn(miso_pin);
    sck = new DigitalOut(sck_pin);
    format(bit_width);

    // mosi_timer_base = pinmap_peripheral(mosi_pin, PinMap_Timer);
    // mosi_pin_mode = pinmap_function(mosi_pin, PinMap_Timer);

    // if (mosi_pin != NC) {
    //     uint32_t pin_number = (uint32_t)mosi_pin - (uint32_t)P0_0;
    //     int index = pin_number >> 4;
    //     int offset = (pin_number & 0xF) << 1;

    //     // PINCONARRAY->PINSEL[index] &= ~(0x3 << offset);
    //     // PINCONARRAY->PINSEL[index] |= mosi_pin_mode << offset;

    //     LOG(INIT,
    //         "MOSI pin channel mapped\r\n"
    //         "  Timer Base:\t0x%08X\r\n"
    //         "  Pin Mode:\t%10u",
    //         mosi_timer_base,
    //         mosi_pin_mode
    //        );
    // } else {
    //     LOG(SEVERE,
    //         "Unable to pinmap MOSI pin!\t(0x%08X)",
    //         mosi_pin
    //        );
    // }

    // sck_timer_base = pinmap_peripheral(sck_pin, PinMap_Timer);
    // sck_pin_mode = pinmap_function(sck_pin, PinMap_Timer);

    // if (sck_pin != NC) {
    //     uint32_t pin_number = (uint32_t)sck_pin - (uint32_t)P0_0;
    //     int index = pin_number >> 4;
    //     int offset = (pin_number & 0xF) << 1;

    //     PINCONARRAY->PINSEL[index] &= ~(0x3 << offset);
    //     PINCONARRAY->PINSEL[index] |= sck_pin_mode << offset;

    //     LOG(INIT,
    //         "SCK pin channel mapped\r\n"
    //         "  Timer Base:\t0x%08X\r\n"
    //         "  Pin Mode:\t%10u",
    //         sck_timer_base,
    //         sck_pin_mode
    //        );
    // } else {
    //     LOG(SEVERE,
    //         "Unable to pinmap SCK pin!\t(0x%08X)",
    //         sck_pin
    //        );
    // }
}

SoftwareSPI::~SoftwareSPI()
{
    delete mosi;
    delete miso;
    delete sck;
}

// void SoftwareSPI::setup()
// {
//     if (mosi_timer_base == sck_timer_base) {
//         setup_timer((LPC_TIM_TypeDef*)sck_timer_base, sck_channel);
//         setup_timer((LPC_TIM_TypeDef*)mosi_timer_base, mosi_channel, true);
//     }
//     // MOSI & SCK must be able to be controlled through a common hardware timer
//     else {
//         LOG(FATAL, "MOSI and SCK do no share the same timer");
//     }
// }

void SoftwareSPI::format(int bits, int mode)
{
    this->bits = bits;
    this->mode = mode;
    polarity = (mode >> 1) & 1;
    phase = mode & 1;
    sck->write(polarity);
}

int SoftwareSPI::write(int value)
{
    int read = 0;

    for (int bit = bits - 1; bit >= 0; --bit) {
        mosi->write( (value >> bit) & 0x01 );

        if (phase == 0) {
            if (miso->read())
                read |= (1 << bit);
        }

        sck->write(!polarity);

        if (phase == 1) {
            if (miso->read())
                read |= (1 << bit);
        }

        sck->write(polarity);
    }

    return read;
}
