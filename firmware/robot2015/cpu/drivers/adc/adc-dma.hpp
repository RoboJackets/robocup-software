#ifdef COMPILE_WITH_DMA

#pragma once

#include "mbed.h"
#include "../../utils/logger/logger.hpp"
#include "MODDMA.hpp"
#include "error_type.hpp"

#define SAMPLE_BUFFER_LENGTH 360

class ADCDMA
{
  public:
    ADCDMA(void);
    ~ADCDMA(void);
    
    ERR_t Init(void);
    bool Check(void);

  protected:
    void TC0_callback(void);
    void ERR0_callback(void);

  private:
    bool isInit;
    bool dmaTransferComplete;

    MODDMA dma;

    uint32_t adc_buf[SAMPLE_BUFFER_LENGTH];
};

#endif
