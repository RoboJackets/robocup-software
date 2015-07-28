#ifdef COMPILE_WITH_DMA

#pragma once

#include "../../config/robot.hpp"
#include "../../utils/robot_types.hpp"


// Make the buffer size match the number of degrees
// in a circle since we are going to output a sinewave.
#define BUFFER_SIZE 360
#define SAMPLES_PER_READ 3
#define DAC_POWER_MODE  (1 << 16)


class Speaker
{
  public:
    Speaker::Speaker(void);
    Speaker::~Speaker(void);


    /**
    * [Speaker::Init Setup the FPGA interface]
    * @return  [The initialization error code.]
    */
    ERR_t Init(void);

  protected:
    void TC0_callback(void);
    void TC1_callback(void);
    void ERR0_callback(void);
    void ERR1_callback(void);

  private:
    bool isInit;

    MODDMA dma;
    MODDMA_Config *conf0, *conf1;

    uint32_t speaker_buf[SAMPLES_PER_READ][BUFFER_SIZE];
    volatile int life_counter;
    void dma_err(void);
};

#endif
