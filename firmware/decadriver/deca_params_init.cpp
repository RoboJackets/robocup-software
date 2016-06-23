/*! ----------------------------------------------------------------------------
 *  @file    deca_params_init.c
 *  @brief   DW1000 configuration parameters
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 *
 * -------------------------------------------------------------------------------------------------------------------
**/
// #include <cstdio>
// #include <cstdlib>

#include "deca_regs.h"
#include "deca_device_api.h"
#include "deca_param_types.h"

#include "mbed.h"

//-----------------------------------------
// map the channel number to the index in the configuration arrays below
// 0th element is chan 1, 1st is chan 2, 2nd is chan 3, 3rd is chan 4, 4th is
// chan 5, 5th is chan 7
const uint8 chan_idx[NUM_CH_SUPPORTED] = {0, 0, 1, 2, 3, 4, 0, 5};

//-----------------------------------------
const uint32 tx_config[NUM_CH] = {
    RF_TXCTRL_CH1, /* Tx value match UM */
    RF_TXCTRL_CH2, RF_TXCTRL_CH3, RF_TXCTRL_CH4, RF_TXCTRL_CH5, RF_TXCTRL_CH7,
};

// RF -> Channel_Specific_Cfg -> Channel_Cfg -> RF_PLL -> RF PLL2
const uint8 pll2_config[NUM_CH][5] = {
    {0x07, 0x04, 0x00, 0x09, 0x1E}, // 3.5Ghz

    {0x08, 0x05, 0x40, 0x08, 0x26}, // 4Ghz

    {0x09, 0x10, 0x40, 0x08, 0x56}, // 4.5Ghz

    {0x08, 0x05, 0x40, 0x08, 0x26}, // 4Ghz WBW

    {0x1D, 0x04, 0x00, 0x08, 0xBE}, // 6.5Ghz

    {0x1D, 0x04, 0x00, 0x08, 0xBE} // 6.5Ghz WBW
};

// bandwidth configuration
const uint8 rx_config[NUM_BW] = {
    0xD8, // NBW
    0xBC // WBW
};

const agc_cfg_struct agc_config = {
    AGC_TUNE2_VAL,

    {AGC_TUNE1_16M, AGC_TUNE1_64M} // adc target
};

const uint8 dwnsSFDlen[NUM_BR] = {
    0x40, 0x10, 0x08}; // DW non-standard SFD length for 110k, 850k and 6.81M

// SFD Threshold
const uint16 sftsh[NUM_BR][NUM_SFD] = {
    // 110k
    {
     (0x0a), // RX_SFTSH_LONG - standard
     (0x16) // RX_SFTSH_USR_LONG - non-standard (DW - length specified above
            // dwnsSFDlen)
    },
    // 850k
    {
     (0x01), // RX_SFTSH_SHORT
     (0x06), // RX_SFTSH_USR_SHORT - non-standard (DW - length specified above
             // dwnsSFDlen)
    },
    // 6.81Mb
    {
     (0x01), // RX_SFTSH_SHORT
     (0x02), // RX_SFTSH_USR_SHORT - non-standard (DW - length specified above
             // dwnsSFDlen)
    }};

const uint16 dtune1[NUM_PRF] = {
    0x0087, // 16 MHz PRF
    0x008D  //  64 MHz PRF
};

const uint32 digital_bb_config[NUM_PRF][NUM_PACS] = {
    // 16 PRF
    {// PAC 8
     0x311A002D,
     // PAC 16
     0x331A0052,
     // PAC 32
     0x351A009A,
     // PAC 64
     0x371A011D},
    // 64 PRF
    {// PAC 8
     0x313B006B,
     // PAC 16
     0x333B00BE,
     // PAC 32
     0x353B015E,
     // PAC 64
     0x373B0296}};

const uint16 lde_replicaCoeff[PCODES] = {

    // 0
    (int)(0.0 * 65536),
    // 1
    (int)(0.35 * 65536),
    // 2
    (int)(0.35 * 65536),
    // 3
    (int)(0.32 * 65536),
    // 4
    (int)(0.26 * 65536),
    // 5
    (int)(0.27 * 65536),
    // 6
    (int)(0.18 * 65536),
    // 7
    (int)(0.50 * 65536),
    // 8
    (int)(0.32 * 65536),
    // 9
    (int)(0.16 * 65536),
    // 10
    (int)(0.20 * 65536),
    // 11
    (int)(0.23 * 65536),
    // 12
    (int)(0.24 * 65536),
    // 13
    (int)(0.23 * 65536),
    // 14
    (int)(0.21 * 65536),
    // 15
    (int)(0.17 * 65536),
    // 16
    (int)(0.21 * 65536),
    // 17
    (int)(0.20 * 65536),
    // 18
    (int)(0.21 * 65536),
    // 19
    (int)(0.21 * 65536),
    // 20
    (int)(0.28 * 65536),
    // 21
    (int)(0.23 * 65536),
    // 22
    (int)(0.22 * 65536),
    // 23
    (int)(0.19 * 65536),
    // 24
    (int)(0.22 * 65536)};
