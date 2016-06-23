/*! ----------------------------------------------------------------------------
 * @file    deca_sleep.c
 * @brief   platform dependent sleep implementation
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include "deca_sleep.h"
#include "mbed.h"

void deca_sleep(unsigned int time_ms) {
  /* This assumes that the tick has a period of exactly one millisecond. See
   * CLOCKS_PER_SEC define. */
  wait_ms(time_ms);
}
