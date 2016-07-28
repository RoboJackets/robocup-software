/*!
 *------------------------------------------------------------------------------------------------------------------
 * @file	deca_device.c
 * @brief	DecaWave device configuration and control functions
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */
#include "mbed.h"
#include "deca_types.h"
#include "deca_param_types.h"
#include "deca_regs.h"
#include "deca_device_api.h"
#include "deca_sleep.h"

// Defines for enable_clocks function
#define FORCE_SYS_XTI 0
#define ENABLE_ALL_SEQ 1
#define FORCE_SYS_PLL 2
#define READ_ACC_ON 7
#define READ_ACC_OFF 8
#define FORCE_OTP_ON 11
#define FORCE_OTP_OFF 12
#define FORCE_TX_PLL 13

// #define DWT_API_ERROR_CHECK     // define so API checks config input
// parameters

// -------------------------------------------------------------------------------------------------------------------
//
// Internal functions for controlling and configuring the device
//
// -------------------------------------------------------------------------------------------------------------------

// Enable and Configure specified clocks
void _dwt_enableclocks(int clocks);
// Configure the ucode (FP algorithm) parameters
void _dwt_configlde(int prf);
// Load ucode from OTP/ROM
void _dwt_loaducodefromrom(void);
// Read non-volatile memory
uint32 _dwt_otpread(uint32 address);
// Program the non-volatile memory
uint32 _dwt_otpprogword32(uint32 data, uint16 address);
// Upload the device configuration into always on memory
void _dwt_aonarrayupload(void);
// -------------------------------------------------------------------------------------------------------------------

/*!
 * Static data for DW1000 DecaWave Transceiver control
 */

// -------------------------------------------------------------------------------------------------------------------
// Structure to hold device data
typedef struct {
  uint32 deviceID;
  uint32 partID;
  uint32 lotID;
  uint8 chan;        // Added channel here - used in the reading of accumulator
  uint8 longFrames;  // Flag in non-standard long frame mode
  uint8 otprev;      // OTP revision number (read during initialisation)
  uint32 txFCTRL;    // Keep TX_FCTRL register config
  uint8 xtrim;       // XTAL trim value read from OTP
  uint8 dblbuffon;   // Double RX buffer mode flag
  uint32 sysCFGreg;  // Local copy of system config register
  uint16 sleep_mode; // Used for automatic reloading of LDO tune and microcode
                     // at wake-up

  dwt_callback_data_t cdata; // Callback data structure

  uint8 wait4resp; // wait4response was set with last TX start command
  int prfIndex;

  void (*dwt_txcallback)(const dwt_callback_data_t *txd);
  void (*dwt_rxcallback)(const dwt_callback_data_t *rxd);

} dwt_local_data_t;

static dwt_local_data_t dw1000local; // Static local device data

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_initialise()
 *
 * @brief This function initiates communications with the DW1000 transceiver
 * and reads its DEV_ID register (address 0x00) to verify the IC is one
 *supported
 * by this software (e.g. DW1000 32-bit device ID value is 0xDECA0130).  Then it
 * does any initial once only device configurations needed for use and
 *initialises
 * as necessary any static data items belonging to this low-level driver.
 *
 * NOTES:
 * 1.this function needs to be run before dwt_configuresleep, also the SPI
 *frequency has to be < 3MHz
 * 2.it also reads and applies LDO tune and crystal trim values from OTP memory
 *
 * input parameters
 * @param config    -   specifies what configuration to load
 *                  DWT_LOADUCODE     0x1 - load the LDE microcode from ROM -
 *enabled accurate RX timestamp
 *                  DWT_LOADNONE      0x0 - do not load any values from OTP
 *memory
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
// OTP addresses definitions
#define LDOTUNE_ADDRESS (0x04)
#define PARTID_ADDRESS (0x06)
#define LOTID_ADDRESS (0x07)
#define VBAT_ADDRESS (0x08)
#define VTEMP_ADDRESS (0x09)
#define XTRIM_ADDRESS (0x1E)

int dwt_initialise(uint16 config) {
  uint8 plllockdetect = EC_CTRL_PLLLCK;
  uint16 otp_addr = 0;
  uint32 ldo_tune = 0;

  dw1000local.dblbuffon = 0;    // Double buffer mode off by default
  dw1000local.prfIndex = 0;     // 16MHz
  dw1000local.cdata.aatset = 0; // Auto ACK bit not set
  dw1000local.wait4resp = 0;
  dw1000local.sleep_mode = 0;

  dw1000local.dwt_txcallback = nullptr;
  dw1000local.dwt_rxcallback = nullptr;

  // Read and validate device ID return -1 if not recognised
  dw1000local.deviceID = dwt_readdevid();
  //Serial pc(USBTX,USBRX);
  //pc.printf("%x\n", dw1000local.deviceID);
  //pc.printf("%x\n", DWT_DEVICE_ID);

  if (DWT_DEVICE_ID !=
      dw1000local.deviceID) // MP IC ONLY (i.e. DW1000) FOR THIS CODE
  {
    return DWT_ERROR;
  }

  //return -2;

  _dwt_enableclocks(FORCE_SYS_XTI); // NOTE: set system clock to XTI - this is
                                    // necessary to make sure the values read by
                                    // _dwt_otpread are reliable

  // Configure the CPLL lock detect
  dwt_writetodevice(EXT_SYNC_ID, EC_CTRL_OFFSET, 1, &plllockdetect);

  // Read OTP revision number
  otp_addr =
      _dwt_otpread(XTRIM_ADDRESS) &
      0xffff; // Read 32 bit value, XTAL trim val is in low octet-0 (5 bits)
  dw1000local.otprev = (otp_addr >> 8) & 0xff; // OTP revision is next byte

  // Load LDO tune from OTP and kick it if there is a value actually programmed.
  ldo_tune = _dwt_otpread(LDOTUNE_ADDRESS);
  if ((ldo_tune & 0xFF) != 0) {
    uint8 ldok = OTP_SF_LDO_KICK;
    // Kick LDO tune
    dwt_writetodevice(OTP_IF_ID, OTP_SF, 1, &ldok); // Set load LDE kick bit
    dw1000local.sleep_mode |=
        AON_WCFG_ONW_LLDO; // LDO tune must be kicked at wake-up
  }

  // Load Part and Lot ID from OTP
  dw1000local.partID = _dwt_otpread(PARTID_ADDRESS);
  dw1000local.lotID = _dwt_otpread(LOTID_ADDRESS);

  // XTAL trim value is set in OTP for DW1000 module and EVK/TREK boards but
  // that might not be the case in a custom design
  dw1000local.xtrim = otp_addr & 0x1F;
  if (!dw1000local
           .xtrim) // A value of 0 means that the crystal has not been trimmed
  {
    dw1000local.xtrim =
        FS_XTALT_MIDRANGE; // Set to mid-range if no calibration value inside
  }
  // Configure XTAL trim
  dwt_xtaltrim(dw1000local.xtrim);

  // Load leading edge detect code
  if (config & DWT_LOADUCODE) {
    _dwt_loaducodefromrom();
    dw1000local.sleep_mode |=
        AON_WCFG_ONW_LLDE; // microcode must be loaded at wake-up
  } else                   // Should disable the LDERUN enable bit in 0x36, 0x4
  {
    uint16 rega = dwt_read16bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET + 1);
    rega &= 0xFDFF; // Clear LDERUN bit
    dwt_write16bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET + 1, rega);
  }

  _dwt_enableclocks(ENABLE_ALL_SEQ); // Enable clocks for sequencing

  // Read system register / store local copy
  dw1000local.sysCFGreg =
      dwt_read32bitreg(SYS_CFG_ID); // Read sysconfig register

  return DWT_SUCCESS;

} // end dwt_initialise()

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_otprevision()
 *
 * @brief This is used to return the read OTP revision
 *
 * input parameters
 *
 * output parameters
 *
 * returns the read OTP revision value
 */
uint8 dwt_otprevision(void) { return dw1000local.otprev; }

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setGPIOforEXTTRX()
 *
 * @brief This is used to enable GPIO for external LNA or PA functionality - HW
 *dependent, consult the DW1000 User Manual
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
void dwt_setGPIOforEXTTRX(void) {
  uint8 buf[GPIO_MODE_LEN];

  // Set the GPIO to control external PA/LNA
  dwt_readfromdevice(GPIO_CTRL_ID, GPIO_MODE_OFFSET, GPIO_MODE_LEN, buf);

  buf[GPIO_LNA_BYTE_NUM] |= (GPIO_PIN5_EXTTXE_8 + GPIO_PIN6_EXTRXE_8);

  dwt_writetodevice(GPIO_CTRL_ID, GPIO_MODE_OFFSET, GPIO_MODE_LEN, buf);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setGPIOdirection()
 *
 * @brief This is used to set GPIO direction as an input (1) or output (0)
 *
 * input parameters
 * @param gpioNum    -   this is the GPIO to configure - see GxM0... GxM8 in the
 *deca_regs.h file
 * @param direction  -   this sets the GPIO direction - see GxP0... GxP8 in the
 *deca_regs.h file
 *
 * output parameters
 *
 * no return value
 */
void dwt_setGPIOdirection(uint32 gpioNum, uint32 direction) {
  uint8 buf[GPIO_DIR_LEN];
  uint32 command = direction | gpioNum;

  buf[0] = command & 0xff;
  buf[1] = (command >> 8) & 0xff;
  buf[2] = (command >> 16) & 0xff;

  dwt_writetodevice(GPIO_CTRL_ID, GPIO_DIR_OFFSET, GPIO_DIR_LEN, buf);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setGPIOvalue()
 *
 * @brief This is used to set GPIO value as (1) or (0) only applies if the GPIO
 *is configured as output
 *
 * input parameters
 * @param gpioNum    -   this is the GPIO to configure - see GxM0... GxM8 in the
 *deca_regs.h file
 * @param value  -   this sets the GPIO value - see GDP0... GDP8 in the
 *deca_regs.h file
 *
 * output parameters
 *
 * no return value
 */
void dwt_setGPIOvalue(uint32 gpioNum, uint32 value) {
  uint8 buf[GPIO_DOUT_LEN];
  uint32 command = value | gpioNum;

  buf[0] = command & 0xff;
  buf[1] = (command >> 8) & 0xff;
  buf[2] = (command >> 16) & 0xff;

  dwt_writetodevice(GPIO_CTRL_ID, GPIO_DOUT_OFFSET, GPIO_DOUT_LEN, buf);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_getpartid()
 *
 * @brief This is used to return the read part ID of the device
 *
 * input parameters
 *
 * output parameters
 *
 * returns the 32 bit part ID value as programmed in the factory
 */
uint32 dwt_getpartid(void) { return dw1000local.partID; }

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_getlotid()
 *
 * @brief This is used to return the read lot ID of the device
 *
 * input parameters
 *
 * output parameters
 *
 * returns the 32 bit lot ID value as programmed in the factory
 */
uint32 dwt_getlotid(void) { return dw1000local.lotID; }

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readdevid()
 *
 * @brief This is used to return the read device type and revision information
 *of the DW1000 device (MP part is 0xDECA0130)
 *
 * input parameters
 *
 * output parameters
 *
 * returns the read value which for DW1000 is 0xDECA0130
 */
uint32 dwt_readdevid(void) { return dwt_read32bitoffsetreg(DEV_ID_ID, 0); }

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_configuretxrf()
 *
 * @brief This function provides the API for the configuration of the TX
 *spectrum
 * including the power and pulse generator delay. The input is a pointer to the
 *data structure
 * of type dwt_txconfig_t that holds all the configurable items.
 *
 * input parameters
 * @param config    -   pointer to the txrf configuration structure, which
 *contains the tx rf config data
 *
 * output parameters
 *
 * no return value
 */
void dwt_configuretxrf(dwt_txconfig_t *config) {

  // Configure RF TX PG_DELAY
  dwt_writetodevice(TX_CAL_ID, TC_PGDELAY_OFFSET, 1, &config->PGdly);

  // Configure TX power
  dwt_write32bitreg(TX_POWER_ID, config->power);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_configure()
 *
 * @brief This function provides the main API for the configuration of the
 * DW1000 and this low-level driver.  The input is a pointer to the data
 *structure
 * of type dwt_config_t that holds all the configurable items.
 * The dwt_config_t structure shows which ones are supported
 *
 * input parameters
 * @param config    -   pointer to the configuration structure, which contains
 *the device configuration data.
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_configure(dwt_config_t *config) {
  uint8 nsSfd_result = 0;
  uint8 useDWnsSFD = 0;
  uint8 chan = config->chan;
  uint32 regval;
  uint16 reg16 = lde_replicaCoeff[config->rxCode];
  uint8 prfIndex = dw1000local.prfIndex = config->prf - DWT_PRF_16M;
  uint8 bw = ((chan == 4) || (chan == 7)) ? 1 : 0; // Select wide or narrow band

  dw1000local.chan = config->chan;

#ifdef DWT_API_ERROR_CHECK
  if (config->dataRate > DWT_BR_6M8) {
    return DWT_ERROR;
  } // validate datarate parameter
  if ((config->prf > DWT_PRF_64M) || (config->prf < DWT_PRF_16M)) {
    return DWT_ERROR; // validate Pulse Repetition Frequency
  }
  if (config->rxPAC > DWT_PAC64) {
    return DWT_ERROR; // validate PAC size
  }
  if ((chan < 1) || (chan > 7) || (6 == chan)) {
    return DWT_ERROR; // validate channel number parameter
  }

  // validate TX and TX pre-amble codes selections
  if (config->prf == DWT_PRF_64M) {
    // at 64M PRF, codes should be 9 to 27 only
    // config->txCode
    // config->rxCode
  } else {
    // at 16M PRF, codes should be 0 to 8 only
  }
  switch (config->txPreambLength) {
  case DWT_PLEN_4096:
  case DWT_PLEN_2048:
  case DWT_PLEN_1536:
  case DWT_PLEN_1024:
  case DWT_PLEN_512:
  case DWT_PLEN_256:
  case DWT_PLEN_128:
  case DWT_PLEN_64:
    break; // all okay
  default:
    return DWT_ERROR; // not a good preamble length parameter
  }

  if (config->phrMode > DWT_PHRMODE_EXT) {
    return DWT_ERROR;
  }
#endif

  // For 110 kbps we need a special setup
  if (DWT_BR_110K == config->dataRate) {
    dw1000local.sysCFGreg |= SYS_CFG_RXM110K;
    reg16 >>= 3; // lde_replicaCoeff must be divided by 8
  } else {
    dw1000local.sysCFGreg &= (~SYS_CFG_RXM110K);
  }

  dw1000local.longFrames = config->phrMode;

  dw1000local.sysCFGreg |= (SYS_CFG_PHR_MODE_11 & (config->phrMode << 16));

  dwt_write32bitreg(SYS_CFG_ID, dw1000local.sysCFGreg);
  // Set the lde_replicaCoeff
  dwt_write16bitoffsetreg(LDE_IF_ID, LDE_REPC_OFFSET, reg16);

  _dwt_configlde(prfIndex);

  // Configure PLL2/RF PLL block CFG (for a given channel)
  dwt_writetodevice(FS_CTRL_ID, FS_PLLCFG_OFFSET, 5,
                    &pll2_config[chan_idx[chan]][0]);

  // Configure RF RX blocks (for specified channel/bandwidth)
  dwt_writetodevice(RF_CONF_ID, RF_RXCTRLH_OFFSET, 1, &rx_config[bw]);

  // Configure RF TX blocks (for specified channel and PRF)
  // Configure RF TX control
  dwt_write32bitoffsetreg(RF_CONF_ID, RF_TXCTRL_OFFSET,
                          tx_config[chan_idx[chan]]);

  // Configure the baseband parameters (for specified PRF, bit rate, PAC, and
  // SFD settings)
  // DTUNE0
  dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_TUNE0b_OFFSET,
                          sftsh[config->dataRate][config->nsSFD]);

  // DTUNE1
  dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_TUNE1a_OFFSET, dtune1[prfIndex]);

  if (config->dataRate == DWT_BR_110K) {
    dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_TUNE1b_OFFSET, 0x64);
  } else {
    if (config->txPreambLength == DWT_PLEN_64) {
      uint8 temp = 0x10;
      dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_TUNE1b_OFFSET, 0x10);
      dwt_writetodevice(DRX_CONF_ID, 0x26, 1, &temp);
    } else {
      uint8 temp = 0x28;
      dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_TUNE1b_OFFSET, 0x20);
      dwt_writetodevice(DRX_CONF_ID, 0x26, 1, &temp);
    }
  }

  // DTUNE2
  dwt_write32bitoffsetreg(DRX_CONF_ID, DRX_TUNE2_OFFSET,
                          digital_bb_config[prfIndex][config->rxPAC]);

  // DTUNE3 (SFD timeout)
  // Don't allow 0 - SFD timeout will always be enabled
  if (config->sfdTO == 0) {
    config->sfdTO = DWT_SFDTOC_DEF;
  }
  dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_SFDTOC_OFFSET, config->sfdTO);

  // Configure AGC parameters
  dwt_write32bitoffsetreg(AGC_CFG_STS_ID, 0xC, agc_config.lo32);
  dwt_write16bitoffsetreg(AGC_CFG_STS_ID, 0x4, agc_config.target[prfIndex]);

  // Set (non-standard) user SFD for improved performance,
  if (config->nsSFD) {
    // Write non standard (DW) SFD length
    dwt_writetodevice(USR_SFD_ID, 0x00, 1, &dwnsSFDlen[config->dataRate]);
    nsSfd_result = 3;
    useDWnsSFD = 1;
  }
  regval = (CHAN_CTRL_TX_CHAN_MASK &
            (chan << CHAN_CTRL_TX_CHAN_SHIFT)) | // Transmit Channel
           (CHAN_CTRL_RX_CHAN_MASK &
            (chan << CHAN_CTRL_RX_CHAN_SHIFT)) | // Receive Channel
           (CHAN_CTRL_RXFPRF_MASK &
            (config->prf << CHAN_CTRL_RXFPRF_SHIFT)) | // RX PRF
           ((CHAN_CTRL_TNSSFD | CHAN_CTRL_RNSSFD) &
            (nsSfd_result << CHAN_CTRL_TNSSFD_SHIFT)) | // nsSFD enable RX&TX
           (CHAN_CTRL_DWSFD &
            (useDWnsSFD << CHAN_CTRL_DWSFD_SHIFT)) | // Use DW nsSFD
           (CHAN_CTRL_TX_PCOD_MASK &
            (config->txCode << CHAN_CTRL_TX_PCOD_SHIFT)) | // TX Preamble Code
           (CHAN_CTRL_RX_PCOD_MASK &
            (config->rxCode << CHAN_CTRL_RX_PCOD_SHIFT)); // RX Preamble Code

  dwt_write32bitreg(CHAN_CTRL_ID, regval);

  // Set up TX Preamble Size and TX PRF
  // Set up TX Ranging Bit and Data Rate
  dw1000local.txFCTRL = (config->txPreambLength | config->prf) << 16;
  dw1000local.txFCTRL |= (config->dataRate << TX_FCTRL_TXBR_SHFT) |
                         TX_FCTRL_TR; // Always set ranging bit !!!
  dwt_write32bitoffsetreg(TX_FCTRL_ID, 0, dw1000local.txFCTRL);

  return DWT_SUCCESS;

} // end dwt_configure()

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setrxantennadelay()
 *
 * @brief This API function writes the antenna delay (in time units) to RX
 *registers
 *
 * input parameters:
 * @param rxDelay - this is the total (RX) antenna delay value, which
 *                          will be programmed into the RX register
 *
 * output parameters
 *
 * no return value
 */
void dwt_setrxantennadelay(uint16 rxDelay) {
  // Set the RX antenna delay for auto TX timestamp adjustment
  dwt_write16bitoffsetreg(LDE_IF_ID, LDE_RXANTD_OFFSET, rxDelay);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_settxantennadelay()
 *
 * @brief This API function writes the antenna delay (in time units) to TX
 *registers
 *
 * input parameters:
 * @param txDelay - this is the total (TX) antenna delay value, which
 *                          will be programmed into the TX delay register
 *
 * output parameters
 *
 * no return value
 */
void dwt_settxantennadelay(uint16 txDelay) {
  // Set the TX antenna delay for auto TX timestamp adjustment
  dwt_write16bitoffsetreg(TX_ANTD_ID, 0x0, txDelay);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_writetxdata()
 *
 * @brief This API function writes the supplied TX data into the DW1000's
 * TX buffer.  The input parameters are the data length in bytes and a pointer
 * to those data bytes.
 *
 * input parameters
 * @param txFrameLength  - This is the total frame length, including the two
 *byte CRC.
 *                         Note: this is the length of TX message (including the
 *2 byte CRC) - max is 1023
 *                         standard PHR mode allows up to 127 bytes
 *                         if > 127 is programmed, DWT_PHRMODE_EXT needs to be
 *set in the phrMode configuration
 *                         see dwt_configure function
 * @param txFrameBytes   - Pointer to the users buffer containing the data to
 *send.
 * @param txBufferOffset - This specifies an offset in the DW1000s TX Buffer at
 *which to start writing data.
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_writetxdata(uint16 txFrameLength, uint8 *txFrameBytes,
                    uint16 txBufferOffset) {
#ifdef DWT_API_ERROR_CHECK
  if (dw1000local.longFrames) {
    if (txFrameLength > 1023) {
      return DWT_ERROR;
    }
  } else {
    if (txFrameLength > 127) {
      return DWT_ERROR;
    }
  }
  if (txFrameLength < 2) {
    return DWT_ERROR;
  }
#endif

  if ((txBufferOffset + txFrameLength) > 1024) {
    return DWT_ERROR;
  }
  // Write the data to the IC TX buffer, (-2 bytes for auto generated CRC)
  dwt_writetodevice(TX_BUFFER_ID, txBufferOffset, txFrameLength - 2,
                    txFrameBytes);

  return DWT_SUCCESS;
} // end dwt_writetxdata()

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_writetxfctrl()
 *
 * @brief This API function configures the TX frame control register before the
 *transmission of a frame
 *
 * input parameters:
 * @param txFrameLength - this is the length of TX message (including the 2 byte
 *CRC) - max is 1023
 *                              NOTE: standard PHR mode allows up to 127 bytes
 *                              if > 127 is programmed, DWT_PHRMODE_EXT needs to
 *be set in the phrMode configuration
 *                              see dwt_configure function
 * @param txBufferOffset - the offset in the tx buffer to start writing the data
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_writetxfctrl(uint16 txFrameLength, uint16 txBufferOffset) {

#ifdef DWT_API_ERROR_CHECK
  if (dw1000local.longFrames) {
    if (txFrameLength > 1023) {
      return DWT_ERROR;
    }
  } else {
    if (txFrameLength > 127) {
      return DWT_ERROR;
    }
  }
  if (txFrameLength < 2) {
    return DWT_ERROR;
  }
#endif

  // Write the frame length to the TX frame control register
  // dw1000local.txFCTRL has kept configured bit rate information
  uint32 reg32 = dw1000local.txFCTRL | txFrameLength | (txBufferOffset << 22);
  dwt_write32bitoffsetreg(TX_FCTRL_ID, 0, reg32);

  return DWT_SUCCESS;

} // end dwt_writetxfctrl()

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readrxdata()
 *
 * @brief This is used to read the data from the RX buffer, from an offset
 *location give by offset parameter
 *
 * input parameters
 * @param buffer - the buffer into which the data will be read
 * @param length - the length of data to read (in bytes)
 * @param rxBufferOffset - the offset in the rx buffer from which to read the
 *data
 *
 * output parameters
 *
 * no return value
 */
void dwt_readrxdata(uint8 *buffer, uint16 length, uint16 rxBufferOffset) {
  dwt_readfromdevice(RX_BUFFER_ID, rxBufferOffset, length, buffer);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readaccdata()
 *
 * @brief This is used to read the data from the Accumulator buffer, from an
 *offset location give by offset parameter
 *
 * input parameters
 * @param buffer - the buffer into which the data will be read
 * @param length - the length of data to read (in bytes)
 * @param accOffset - the offset in the acc buffer from which to read the data
 *
 * output parameters
 *
 * no return value
 */
void dwt_readaccdata(uint8 *buffer, uint16 len, uint16 accOffset) {
  // Force on the ACC clocks if we are sequenced
  _dwt_enableclocks(READ_ACC_ON);

  dwt_readfromdevice(ACC_MEM_ID, accOffset, len, buffer);

  _dwt_enableclocks(READ_ACC_OFF); // Revert clocks back
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readdiagnostics()
 *
 * @brief this function reads the RX signal quality diagnostic data
 *
 * input parameters
 * @param diagnostics - diagnostic structure pointer, this will contain the
 *diagnostic data read from the DW1000
 *
 * output parameters
 *
 * no return value
 */
void dwt_readdiagnostics(dwt_rxdiag_t *diagnostics) {
  // Read the HW FP index
  diagnostics->firstPath =
      dwt_read16bitoffsetreg(RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET);

  // LDE diagnostic data
  diagnostics->maxNoise = dwt_read16bitoffsetreg(LDE_IF_ID, LDE_THRESH_OFFSET);

  // Read all 8 bytes in one SPI transaction
  dwt_readfromdevice(RX_FQUAL_ID, 0x0, 8, (uint8 *)&diagnostics->stdNoise);
  // diagnostics->stdNoise = dwt_read16bitoffsetreg(RX_FQUAL_ID, 0x0) ;
  // diagnostics->firstPathAmp2 = dwt_read16bitoffsetreg(RX_FQUAL_ID, 0x2) ;
  // diagnostics->firstPathAmp3 = dwt_read16bitoffsetreg(RX_FQUAL_ID, 0x4) ;
  // diagnostics->maxGrowthCIR = dwt_read16bitoffsetreg(RX_FQUAL_ID, 0x6) ;

  diagnostics->firstPathAmp1 = dwt_read16bitoffsetreg(RX_TIME_ID, 0x7);

  diagnostics->rxPreamCount =
      (dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXPACC_MASK) >>
      RX_FINFO_RXPACC_SHIFT;

  // diagnostics->debug1 = dwt_read32bitoffsetreg(0x27, 0x28);
  // diagnostics->debug2 = dwt_read32bitoffsetreg(0x27, 0xc);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readtxtimestamp()
 *
 * @brief This is used to read the TX timestamp (adjusted with the programmed
 *antenna delay)
 *
 * input parameters
 * @param timestamp - a pointer to a 5-byte buffer which will store the read TX
 *timestamp time
 *
 * output parameters - the timestamp buffer will contain the value after the
 *function call
 *
 * no return value
 */
void dwt_readtxtimestamp(uint8 *timestamp) {
  dwt_readfromdevice(TX_TIME_ID, 0, TX_TIME_TX_STAMP_LEN,
                     timestamp); // Read bytes directly into buffer
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readtxtimestamphi32()
 *
 * @brief This is used to read the high 32-bits of the TX timestamp (adjusted
 *with the programmed antenna delay)
 *
 * input parameters
 *
 * output parameters
 *
 * returns high 32-bits of TX timestamp
 */
uint32 dwt_readtxtimestamphi32(void) {
  return dwt_read32bitoffsetreg(TX_TIME_ID, 1);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readtxtimestamplo32()
 *
 * @brief This is used to read the low 32-bits of the TX timestamp (adjusted
 *with the programmed antenna delay)
 *
 * input parameters
 *
 * output parameters
 *
 * returns low 32-bits of TX timestamp
 */
uint32 dwt_readtxtimestamplo32(void) {
  return dwt_read32bitoffsetreg(TX_TIME_ID, 0);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readrxtimestamp()
 *
 * @brief This is used to read the RX timestamp (adjusted time of arrival)
 *
 * input parameters
 * @param timestamp - a pointer to a 5-byte buffer which will store the read RX
 *timestamp time
 *
 * output parameters - the timestamp buffer will contain the value after the
 *function call
 *
 * no return value
 */
void dwt_readrxtimestamp(uint8 *timestamp) {
  dwt_readfromdevice(RX_TIME_ID, 0, RX_TIME_RX_STAMP_LEN,
                     timestamp); // Get the adjusted time of arrival
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readrxtimestamphi32()
 *
 * @brief This is used to read the high 32-bits of the RX timestamp (adjusted
 *with the programmed antenna delay)
 *
 * input parameters
 *
 * output parameters
 *
 * returns high 32-bits of RX timestamp
 */
uint32 dwt_readrxtimestamphi32(void) {
  return dwt_read32bitoffsetreg(RX_TIME_ID, 1);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readrxtimestamplo32()
 *
 * @brief This is used to read the low 32-bits of the RX timestamp (adjusted
 *with the programmed antenna delay)
 *
 * input parameters
 *
 * output parameters
 *
 * returns low 32-bits of RX timestamp
 */
uint32 dwt_readrxtimestamplo32(void) {
  return dwt_read32bitoffsetreg(RX_TIME_ID, 0);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readsystimestamphi32()
 *
 * @brief This is used to read the high 32-bits of the system time
 *
 * input parameters
 *
 * output parameters
 *
 * returns high 32-bits of system time timestamp
 */
uint32 dwt_readsystimestamphi32(void) {
  return dwt_read32bitoffsetreg(SYS_TIME_ID, 1);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readsystime()
 *
 * @brief This is used to read the system time
 *
 * input parameters
 * @param timestamp - a pointer to a 5-byte buffer which will store the read
 *system time
 *
 * output parameters
 * @param timestamp - the timestamp buffer will contain the value after the
 *function call
 *
 * no return value
 */
void dwt_readsystime(uint8 *timestamp) {
  dwt_readfromdevice(SYS_TIME_ID, 0, SYS_TIME_LEN, timestamp);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_writetodevice()
 *
 * @brief  this function is used to write to the DW1000 device registers
 * Notes:
 *        1. Firstly we create a header (the first byte is a header byte)
 *        a. check if sub index is used, if subindexing is used - set bit-6 to 1
 *to signify that the sub-index address follows the register index byte
 *        b. set bit-7 (or with 0x80) for write operation
 *        c. if extended sub address index is used (i.e. if index > 127) set
 *bit-7 of the first sub-index byte following the first header byte
 *
 *        2. Write the header followed by the data bytes to the DW1000 device
 *
 *
 * input parameters:
 * @param recordNumber  - ID of register file or buffer being accessed
 * @param index         - byte index into register file or buffer being accessed
 * @param length        - number of bytes being written
 * @param buffer        - pointer to buffer containing the 'length' bytes to be
 *written
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_writetodevice(uint16 recordNumber, uint16 index, uint32 length,
                      const uint8 *buffer) {
  uint8 header[3]; // Buffer to compose header in
  int cnt = 0;     // Counter for length of header
#ifdef DWT_API_ERROR_CHECK
  if (recordNumber > 0x3F) {
    return DWT_ERROR; // Record number is limited to 6-bits.
  }
#endif

  // Write message header selecting WRITE operation and addresses as appropriate
  // (this is one to three bytes long)
  if (index == 0) // For index of 0, no sub-index is required
  {
    header[cnt++] = 0x80 | recordNumber; // Bit-7 is WRITE operation, bit-6
                                         // zero=NO sub-addressing, bits 5-0 is
                                         // reg file id
  } else {
#ifdef DWT_API_ERROR_CHECK
    if (index > 0x7FFF) {
      return DWT_ERROR; // Index is limited to 15-bits.
    }
    if ((index + length) > 0x7FFF) {
      return DWT_ERROR; // Sub-addressable area is limited to 15-bits.
    }
#endif
    header[cnt++] = 0xC0 | recordNumber; // Bit-7 is WRITE operation, bit-6
                                         // one=sub-address follows, bits 5-0 is
                                         // reg file id

    if (index <= 127) // For non-zero index < 127, just a single sub-index byte
                      // is required
    {
      header[cnt++] =
          (uint8)index; // Bit-7 zero means no extension, bits 6-0 is index.
    } else {
      header[cnt++] = 0x80 | (uint8)(index); // Bit-7 one means extended index,
                                             // bits 6-0 is low seven bits of
                                             // index.
      header[cnt++] =
          (uint8)(index >> 7); // 8-bit value = high eight bits of index.
    }
  }

  // Write it to the SPI
  return writetospi(cnt, header, length, buffer);

} // end dwt_writetodevice()

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readfromdevice()
 *
 * @brief  this function is used to read from the DW1000 device registers
 * Notes:
 *        1. Firstly we create a header (the first byte is a header byte)
 *        a. check if sub index is used, if subindexing is used - set bit-6 to 1
 *to signify that the sub-index address follows the register index byte
 *        b. set bit-7 (or with 0x80) for write operation
 *        c. if extended sub address index is used (i.e. if index > 127) set
 *bit-7 of the first sub-index byte following the first header byte
 *
 *        2. Write the header followed by the data bytes to the DW1000 device
 *        3. Store the read data in the input buffer
 *
 * input parameters:
 * @param recordNumber  - ID of register file or buffer being accessed
 * @param index         - byte index into register file or buffer being accessed
 * @param length        - number of bytes being read
 * @param buffer        - pointer to buffer in which to return the read data.
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_readfromdevice(uint16 recordNumber, uint16 index, uint32 length,
                       uint8 *buffer) {
  uint8 header[3]; // Buffer to compose header in
  int cnt = 0;     // Counter for length of header
#ifdef DWT_API_ERROR_CHECK
  if (recordNumber > 0x3F) {
    return DWT_ERROR; // Record number is limited to 6-bits.
  }
#endif

  // Write message header selecting READ operation and addresses as appropriate
  // (this is one to three bytes long)
  if (index == 0) // For index of 0, no sub-index is required
  {
    header[cnt++] = (uint8)recordNumber; // Bit-7 zero is READ operation, bit-6
                                         // zero=NO sub-addressing, bits 5-0 is
                                         // reg file id
  } else {
#ifdef DWT_API_ERROR_CHECK
    if (index > 0x7FFF) {
      return DWT_ERROR; // Index is limited to 15-bits.
    }
    if ((index + length) > 0x7FFF) {
      return DWT_ERROR; // Sub-addressable area is limited to 15-bits.
    }
#endif
    header[cnt++] = (uint8)(0x40 | recordNumber); // Bit-7 zero is READ
                                                  // operation, bit-6
                                                  // one=sub-address follows,
                                                  // bits 5-0 is reg file id

    if (index <= 127) // For non-zero index < 127, just a single sub-index byte
                      // is required
    {
      header[cnt++] =
          (uint8)index; // Bit-7 zero means no extension, bits 6-0 is index.
    } else {
      header[cnt++] = 0x80 | (uint8)(index); // Bit-7 one means extended index,
                                             // bits 6-0 is low seven bits of
                                             // index.
      header[cnt++] =
          (uint8)(index >> 7); // 8-bit value = high eight bits of index.
    }
  }

  // Do the read from the SPI
  return readfromspi(cnt, header, length,
                     buffer); // result is stored in the buffer

} // end dwt_readfromdevice()

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_read32bitoffsetreg()
 *
 * @brief  this function is used to read 32-bit value from the DW1000 device
 *registers
 *
 * input parameters:
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 *
 * output parameters
 *
 * returns 32 bit register value (success), or DWT_ERROR for error
 */
uint32 dwt_read32bitoffsetreg(int regFileID, int regOffset) {
  uint32 regval = DWT_ERROR;
  int j;
  uint8 buffer[4];

  int result =
      dwt_readfromdevice(regFileID, regOffset, 4,
                         buffer); // Read 4 bytes (32-bits) register into buffer

  if (result == DWT_SUCCESS) {
    for (j = 3; j >= 0; j--) {
      regval = (regval << 8) + buffer[j];
    }
  }
  return regval;

} // end dwt_read32bitoffsetreg()

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_read16bitoffsetreg()
 *
 * @brief  this function is used to read 16-bit value from the DW1000 device
 *registers
 *
 * input parameters:
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 *
 * output parameters
 *
 * returns 16 bit register value (success), or DWT_ERROR for error
 */
uint16 dwt_read16bitoffsetreg(int regFileID, int regOffset) {
  uint16 regval = DWT_ERROR;
  uint8 buffer[2];

  int result =
      dwt_readfromdevice(regFileID, regOffset, 2,
                         buffer); // Read 2 bytes (16-bits) register into buffer

  if (result == DWT_SUCCESS) {
    regval = (buffer[1] << 8) + buffer[0];
  }
  return regval;

} // end dwt_read16bitoffsetreg()

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_write16bitoffsetreg()
 *
 * @brief  this function is used to write 16-bit value to the DW1000 device
 *registers
 *
 * input parameters:
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 * @param regval    - the value to write
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_write16bitoffsetreg(int regFileID, int regOffset, uint16 regval) {
  int reg;
  uint8 buffer[2];

  buffer[0] = regval & 0xFF;
  buffer[1] = regval >> 8;

  reg = dwt_writetodevice(regFileID, regOffset, 2, buffer);

  return reg;

} // end dwt_write16bitoffsetreg()

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_write32bitoffsetreg()
 *
 * @brief  this function is used to write 32-bit value to the DW1000 device
 *registers
 *
 * input parameters:
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 * @param regval    - the value to write
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_write32bitoffsetreg(int regFileID, int regOffset, uint32 regval) {
  int j;
  int reg;
  uint8 buffer[4];

  for (j = 0; j < 4; j++) {
    buffer[j] = regval & 0xff;
    regval >>= 8;
  }

  reg = dwt_writetodevice(regFileID, regOffset, 4, buffer);

  return reg;

} // end dwt_write32bitoffsetreg()

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_enableframefilter()
 *
 * @brief This is used to enable the frame filtering - (the default option is to
 * accept any data and ACK frames with correct destination address
 *
 * input parameters
 * @param - bitmask - enables/disables the frame filtering options according to
 *      DWT_FF_NOTYPE_EN        0x000   no frame types allowed
 *      DWT_FF_COORD_EN         0x002   behave as coordinator (can receive
 *frames with no destination address (PAN ID has to match))
 *      DWT_FF_BEACON_EN        0x004   beacon frames allowed
 *      DWT_FF_DATA_EN          0x008   data frames allowed
 *      DWT_FF_ACK_EN           0x010   ack frames allowed
 *      DWT_FF_MAC_EN           0x020   mac control frames allowed
 *      DWT_FF_RSVD_EN          0x040   reserved frame types allowed
 *
 * output parameters
 *
 * no return value
 */
void dwt_enableframefilter(uint16 enable) {
  uint32 sysconfig =
      SYS_CFG_MASK & dwt_read32bitreg(SYS_CFG_ID); // Read sysconfig register

  if (enable) {
    // Enable frame filtering and configure frame types
    sysconfig &= ~(SYS_CFG_FF_ALL_EN); // Clear all
    sysconfig |= (enable & SYS_CFG_FF_ALL_EN) | SYS_CFG_FFE;
  } else {
    sysconfig &= ~(SYS_CFG_FFE);
  }

  dw1000local.sysCFGreg = sysconfig;
  dwt_write32bitreg(SYS_CFG_ID, sysconfig);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setpanid()
 *
 * @brief This is used to set the PAN ID
 *
 * input parameters
 * @param panID - this is the PAN ID
 *
 * output parameters
 *
 * no return value
 */
void dwt_setpanid(uint16 panID) {
  // PAN ID is high 16 bits of register
  dwt_write16bitoffsetreg(PANADR_ID, 2, panID);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setaddress16()
 *
 * @brief This is used to set 16-bit (short) address
 *
 * input parameters
 * @param shortAddress - this sets the 16 bit short address
 *
 * output parameters
 *
 * no return value
 */
void dwt_setaddress16(uint16 shortAddress) {
  // Short address into low 16 bits
  dwt_write16bitoffsetreg(PANADR_ID, 0, shortAddress);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_seteui()
 *
 * @brief This is used to set the EUI 64-bit (long) address
 *
 * input parameters
 * @param eui64 - this is the pointer to a buffer that contains the 64bit
 *address
 *
 * output parameters
 *
 * no return value
 */
void dwt_seteui(uint8 *eui64) { dwt_writetodevice(EUI_64_ID, 0x0, 8, eui64); }

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_geteui()
 *
 * @brief This is used to get the EUI 64-bit from the DW1000
 *
 * input parameters
 * @param eui64 - this is the pointer to a buffer that will contain the read
 *64-bit EUI value
 *
 * output parameters
 *
 * no return value
 */
void dwt_geteui(uint8 *eui64) { dwt_readfromdevice(EUI_64_ID, 0x0, 8, eui64); }

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_otpread()
 *
 * @brief This is used to read the OTP data from given address into provided
 *array
 *
 * input parameters
 * @param address - this is the OTP address to read from
 * @param array - this is the pointer to the array into which to read the data
 * @param length - this is the number of 32 bit words to read (array needs to be
 *at least this length)
 *
 * output parameters
 *
 * no return value
 */
void dwt_otpread(uint32 address, uint32 *array, uint8 length) {
  int i;

  _dwt_enableclocks(FORCE_SYS_XTI); // NOTE: Set system clock to XTAL - this is
                                    // necessary to make sure the values read by
                                    // _dwt_otpread are reliable

  for (i = 0; i < length; i++) {
    array[i] = _dwt_otpread(address + i);
  }

  _dwt_enableclocks(ENABLE_ALL_SEQ); // Restore system clock to PLL

  return;
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_otpread()
 *
 * @brief function to read the OTP memory. Ensure that MR,MRa,MRb are reset to
 *0.
 *
 * input parameters
 * @param address - address to read at
 *
 * output parameters
 *
 * returns the 32bit of read data
 */
uint32 _dwt_otpread(uint32 address) {
  uint8 buf[4];
  uint32 ret_data;

  buf[1] = (address >> 8) & 0xff;
  buf[0] = address & 0xff;

  // Write the address
  dwt_writetodevice(OTP_IF_ID, OTP_ADDR, 2, buf);

  // Assert OTP Read (self clearing)
  buf[0] = 0x03; // 0x03 for manual drive of OTP_READ
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, buf);
  buf[0] = 0x00; // Bit0 is not autoclearing, so clear it (Bit 1 is but we clear
                 // it anyway).
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, buf);

  // Read read data, available 40ns after rising edge of OTP_READ
  ret_data = dwt_read32bitoffsetreg(OTP_IF_ID, OTP_RDAT);

  // Return the 32bit of read data
  return (ret_data);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_otpsetmrregs()
 *
 * @brief Configure the MR registers for initial programming (enable charge
 *pump).
 * Read margin is used to stress the read back from the
 * programmed bit. In normal operation this is relaxed.
 *
 * input parameters
 * @param mode - "0" : Reset all to 0x0:           MRA=0x0000, MRB=0x0000,
 *MR=0x0000
 *               "1" : Set for inital programming: MRA=0x9220, MRB=0x000E,
 *MR=0x1024
 *               "2" : Set for soak programming:   MRA=0x9220, MRB=0x0003,
 *MR=0x1824
 *               "3" : High Vpp:                   MRA=0x9220, MRB=0x004E,
 *MR=0x1824
 *               "4" : Low Read Margin:            MRA=0x0000, MRB=0x0003,
 *MR=0x0000
 *               "5" : Array Clean:                MRA=0x0049, MRB=0x0003,
 *MR=0x0024
 *               "4" : Very Low Read Margin:       MRA=0x0000, MRB=0x0003,
 *MR=0x0000
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
uint32 _dwt_otpsetmrregs(int mode) {
  uint8 rd_buf[4];
  uint8 wr_buf[4];
  uint32 mra = 0, mrb = 0, mr = 0;
  // printf("OTP SET MR: Setting MR,MRa,MRb for mode %2x\n",mode);

  // PROGRAMME MRA
  // Set MRA, MODE_SEL
  wr_buf[0] = 0x03;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL + 1, 1, wr_buf);

  // Load data
  switch (mode & 0x0f) {
  case 0x0:
    mr = 0x0000;
    mra = 0x0000;
    mrb = 0x0000;
    break;
  case 0x1:
    mr = 0x1024;
    mra = 0x9220; // Enable CPP mon
    mrb = 0x000e;
    break;
  case 0x2:
    mr = 0x1824;
    mra = 0x9220;
    mrb = 0x0003;
    break;
  case 0x3:
    mr = 0x1824;
    mra = 0x9220;
    mrb = 0x004e;
    break;
  case 0x4:
    mr = 0x0000;
    mra = 0x0000;
    mrb = 0x0003;
    break;
  case 0x5:
    mr = 0x0024;
    mra = 0x0000;
    mrb = 0x0003;
    break;
  default:
    //  printf("OTP SET MR: ERROR : Invalid mode selected\n",mode);
    return DWT_ERROR;
  }

  wr_buf[0] = mra & 0x00ff;
  wr_buf[1] = (mra & 0xff00) >> 8;
  dwt_writetodevice(OTP_IF_ID, OTP_WDAT, 2, wr_buf);

  // Set WRITE_MR
  wr_buf[0] = 0x08;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);

  // Wait?

  // Set Clear Mode sel
  wr_buf[0] = 0x02;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL + 1, 1, wr_buf);

  // Set AUX update, write MR
  wr_buf[0] = 0x88;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);
  // Clear write MR
  wr_buf[0] = 0x80;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);
  // Clear AUX update
  wr_buf[0] = 0x00;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);

  ///////////////////////////////////////////
  // PROGRAM MRB
  // Set SLOW, MRB, MODE_SEL
  wr_buf[0] = 0x05;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL + 1, 1, wr_buf);

  wr_buf[0] = mrb & 0x00ff;
  wr_buf[1] = (mrb & 0xff00) >> 8;
  dwt_writetodevice(OTP_IF_ID, OTP_WDAT, 2, wr_buf);

  // Set WRITE_MR
  wr_buf[0] = 0x08;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);

  // Wait?

  // Set Clear Mode sel
  wr_buf[0] = 0x04;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL + 1, 1, wr_buf);

  // Set AUX update, write MR
  wr_buf[0] = 0x88;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);
  // Clear write MR
  wr_buf[0] = 0x80;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);
  // Clear AUX update
  wr_buf[0] = 0x00;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);

  ///////////////////////////////////////////
  // PROGRAM MR
  // Set SLOW, MODE_SEL
  wr_buf[0] = 0x01;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL + 1, 1, wr_buf);
  // Load data

  wr_buf[0] = mr & 0x00ff;
  wr_buf[1] = (mr & 0xff00) >> 8;
  dwt_writetodevice(OTP_IF_ID, OTP_WDAT, 2, wr_buf);

  // Set WRITE_MR
  wr_buf[0] = 0x08;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);

  // Wait?
  deca_sleep(10);
  // Set Clear Mode sel
  wr_buf[0] = 0x00;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL + 1, 1, wr_buf);

  // Read confirm mode writes.
  // Set man override, MRA_SEL
  wr_buf[0] = OTP_CTRL_OTPRDEN;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);
  wr_buf[0] = 0x02;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL + 1, 1, wr_buf);
  // MRB_SEL
  wr_buf[0] = 0x04;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL + 1, 1, wr_buf);
  deca_sleep(100);

  // Clear mode sel
  wr_buf[0] = 0x00;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL + 1, 1, wr_buf);
  // Clear MAN_OVERRIDE
  wr_buf[0] = 0x00;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);

  deca_sleep(10);

  if (((mode & 0x0f) == 0x1) || ((mode & 0x0f) == 0x2)) {
    // Read status register
    dwt_readfromdevice(OTP_IF_ID, OTP_STAT, 1, rd_buf);
  }

  return DWT_SUCCESS;
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_otpprogword32()
 *
 * @brief function to program the OTP memory. Ensure that MR,MRa,MRb are reset
 *to 0.
 * VNM Charge pump needs to be enabled (see _dwt_otpsetmrregs)
 * Note the address is only 11 bits long.
 *
 * input parameters
 * @param address - address to read at
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
uint32 _dwt_otpprogword32(uint32 data, uint16 address) {
  uint8 rd_buf[1];
  uint8 wr_buf[4];
  uint8 otp_done;

  // Read status register
  dwt_readfromdevice(OTP_IF_ID, OTP_STAT, 1, rd_buf);

  if ((rd_buf[0] & 0x02) != 0x02) {
    //        printf("OTP PROG 32: ERROR VPP NOT OK, programming will fail. Are
    //        MR/MRA/MRB set?\n");
    return DWT_ERROR;
  }

  // Write the data
  wr_buf[3] = (data >> 24) & 0xff;
  wr_buf[2] = (data >> 16) & 0xff;
  wr_buf[1] = (data >> 8) & 0xff;
  wr_buf[0] = data & 0xff;
  dwt_writetodevice(OTP_IF_ID, OTP_WDAT, 4, wr_buf);

  // Write the address [10:0]
  wr_buf[1] = (address >> 8) & 0x07;
  wr_buf[0] = address & 0xff;
  dwt_writetodevice(OTP_IF_ID, OTP_ADDR, 2, wr_buf);

  // Enable Sequenced programming
  wr_buf[0] = OTP_CTRL_OTPPROG;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);
  wr_buf[0] = 0x00; // And clear
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);

  // WAIT for status to flag PRGM OK..
  otp_done = 0;
  while (otp_done == 0) {
    deca_sleep(1);
    dwt_readfromdevice(OTP_IF_ID, OTP_STAT, 1, rd_buf);

    if ((rd_buf[0] & 0x01) == 0x01) {
      otp_done = 1;
    }
  }

  return DWT_SUCCESS;
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_otpwriteandverify()
 *
 * @brief This is used to program 32-bit value into the DW1000 OTP memory.
 *
 * input parameters
 * @param value - this is the 32-bit value to be programmed into OTP
 * @param address - this is the 16-bit OTP address into which the 32-bit value
 *is programmed
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
uint32 dwt_otpwriteandverify(uint32 value, uint16 address) {
  int prog_ok = DWT_SUCCESS;
  int retry = 0;
  // Firstly set the system clock to crystal
  _dwt_enableclocks(FORCE_SYS_XTI); // set system clock to XTI

  //
  //!!!!!!!!!!!!!! NOTE !!!!!!!!!!!!!!!!!!!!!
  // Set the supply to 3.7V
  //

  _dwt_otpsetmrregs(1); // Set mode for programming

  // For each value to program - the readback/check is done couple of times to
  // verify it has programmed successfully
  while (1) {
    _dwt_otpprogword32(value, address);

    if (_dwt_otpread(address) == value) {
      break;
    }
    retry++;
    if (retry == 5) {
      break;
    }
  }

  // Even if the above does not exit before retry reaches 5, the programming has
  // probably been successful

  _dwt_otpsetmrregs(4); // Set mode for reading

  if (_dwt_otpread(address) !=
      value) // If this does not pass please check voltage supply on VDDIO
  {
    prog_ok = DWT_ERROR;
  }

  _dwt_otpsetmrregs(0); // Setting OTP mode register for low RM read - resetting
                        // the device would be alternative

  return prog_ok;
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_aonconfigupload()
 *
 * @brief This function uploads always on (AON) configuration, as set in the
 *AON_CFG0_OFFSET register.
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
void _dwt_aonconfigupload(void) {
  uint8 buf[1];

  buf[0] = 0x04;
  dwt_writetodevice(AON_ID, AON_CTRL_OFFSET, 1, buf);
  buf[0] = 0x00;
  dwt_writetodevice(AON_ID, AON_CTRL_OFFSET, 1, buf);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_aonarrayupload()
 *
 * @brief This function uploads always on (AON) data array and configuration.
 *Thus if this function is used, then _dwt_aonconfigupload
 * is not necessary. The DW1000 will go so SLEEP straight after this if the
 *DWT_SLP_EN has been set.
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
void _dwt_aonarrayupload(void) {
  uint8 buf[1];

  buf[0] = 0x00;
  dwt_writetodevice(AON_ID, AON_CTRL_OFFSET, 1, buf);
  buf[0] = 0x02;
  dwt_writetodevice(AON_ID, AON_CTRL_OFFSET, 1, buf);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_entersleep()
 *
 * @brief This function puts the device into deep sleep or sleep.
 *dwt_configuredeepsleep should be called first
 * to configure the sleep and on-wake/wake-up parameters
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
void dwt_entersleep(void) {
  // Copy config to AON - upload the new configuration
  _dwt_aonarrayupload();
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_configuresleepcnt()
 *
 * @brief sets the sleep counter to new value, this function programs the high
 *16-bits of the 28-bit counter
 *
 * NOTE: this function needs to be run before dwt_configuresleep, also the SPI
 *frequency has to be < 3MHz
 *
 * input parameters
 * @param sleepcnt - this it value of the sleep counter to program
 *
 * output parameters
 *
 * no return value
 */
void dwt_configuresleepcnt(uint16 sleepcnt) {
  uint8 buf[2];

  buf[0] = 0x01;
  dwt_writetodevice(PMSC_ID, PMSC_CTRL0_OFFSET, 1, buf);

  buf[0] = 0;
  dwt_writetodevice(AON_ID, AON_CFG0_OFFSET, 1,
                    buf); // To make sure we don't accidentaly go to sleep

  buf[0] = 0;
  dwt_writetodevice(AON_ID, AON_CFG1_OFFSET, 1, buf);
  // Disable the sleep counter
  _dwt_aonconfigupload();

  // Set new value
  buf[0] = sleepcnt & 0xFF;
  buf[1] = (sleepcnt >> 8) & 0xFF;
  dwt_writetodevice(AON_ID, (AON_CFG0_OFFSET + 2), 2, buf);
  _dwt_aonconfigupload();

  // Enable the sleep counter
  buf[0] = 1;
  dwt_writetodevice(AON_ID, AON_CFG1_OFFSET, 1, buf);
  _dwt_aonconfigupload();

  buf[0] = 0x00;
  dwt_writetodevice(PMSC_ID, PMSC_CTRL0_OFFSET, 1, buf);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_calibratesleepcnt()
 *
 * @brief calibrates the local oscillator as its frequency can vary between 7
 *and 13kHz depending on temp and voltage
 *
 * NOTE: this function needs to be run before dwt_configuresleepcnt, so that we
 *know what the counter units are
 *
 * input parameters
 *
 * output parameters
 *
 * returns the number of XTAL/2 cycles per low-power oscillator cycle. LP OSC
 *frequency = 19.2 MHz/return value
 */
uint16 dwt_calibratesleepcnt(void) {
  uint8 buf[2];
  uint16 result;

  // Enable cal of the sleep counter
  buf[0] = 4;
  dwt_writetodevice(AON_ID, AON_CFG1_OFFSET, 1, buf);
  _dwt_aonconfigupload();

  // Disables the cal of the sleep counter
  buf[0] = 0;
  dwt_writetodevice(AON_ID, AON_CFG1_OFFSET, 1, buf);
  _dwt_aonconfigupload();

  buf[0] = 0x01;
  dwt_writetodevice(PMSC_ID, PMSC_CTRL0_OFFSET, 1, buf);
  deca_sleep(1);

  // Read the number of XTAL/2 cycles one lposc cycle took.
  // Set up address
  buf[0] = 118;
  dwt_writetodevice(AON_ID, AON_ADDR_OFFSET, 1, buf);

  // Enable manual override
  buf[0] = 0x80; // OVR EN
  dwt_writetodevice(AON_ID, AON_CTRL_OFFSET, 1, buf);

  // Read confirm data that was written
  buf[0] = 0x88; // OVR EN, OVR_RD
  dwt_writetodevice(AON_ID, AON_CTRL_OFFSET, 1, buf);

  // Read back byte from AON
  dwt_readfromdevice(AON_ID, AON_RDAT_OFFSET, 1, buf);
  result = buf[0];
  result = result << 8;

  // Set up address
  buf[0] = 117;
  dwt_writetodevice(AON_ID, AON_ADDR_OFFSET, 1, buf);

  // Enable manual override
  buf[0] = 0x80; // OVR EN
  dwt_writetodevice(AON_ID, AON_CTRL_OFFSET, 1, buf);

  // Read confirm data that was written
  buf[0] = 0x88; // OVR EN, OVR_RD
  dwt_writetodevice(AON_ID, AON_CTRL_OFFSET, 1, buf);

  // Read back byte from AON
  dwt_readfromdevice(AON_ID, AON_RDAT_OFFSET, 1, buf);
  result |= buf[0];

  buf[0] = 0x00; // Disable OVR EN
  dwt_writetodevice(AON_ID, AON_CTRL_OFFSET, 1, buf);

  buf[0] = 0x00;
  dwt_writetodevice(PMSC_ID, PMSC_CTRL0_OFFSET, 1, buf);

  // Returns the number of XTAL/2 cycles per one LP OSC cycle
  // This can be converted into LP OSC frequency by 19.2 MHz/result
  return result;
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_configuresleep()
 *
 * @brief configures the device for both DEEP_SLEEP and SLEEP modes, and on-wake
 *mode
 * i.e. before entering the sleep, the device should be programmed for TX or RX,
 *then upon "waking up" the TX/RX settings
 * will be preserved and the device can immediately perform the desired action
 *TX/RX
 *
 * NOTE: e.g. Tag operation - after deep sleep, the device needs to just load
 *the TX buffer and send the frame
 *
 *
 *      mode: the array and LDE code (OTP/ROM) and LDO tune, and set sleep
 *persist
 *      DWT_PRESRV_SLEEP 0x0100 - preserve sleep
 *      DWT_LOADOPSET    0x0080 - load operating parameter set on wakeup
 *      DWT_CONFIG       0x0040 - download the AON array into the HIF
 *(configuration download)
 *      DWT_LOADEUI      0x0008
 *      DWT_GOTORX       0x0002
 *      DWT_TANDV        0x0001
 *
 *      wake: wake up parameters
 *      DWT_XTAL_EN      0x10 - keep XTAL running during sleep
 *      DWT_WAKE_SLPCNT  0x8 - wake up after sleep count
 *      DWT_WAKE_CS      0x4 - wake up on chip select
 *      DWT_WAKE_WK      0x2 - wake up on WAKEUP PIN
 *      DWT_SLP_EN       0x1 - enable sleep/deep sleep functionality
 *
 * input parameters
 * @param mode - config on-wake parameters
 * @param wake - config wake up parameters
 *
 * output parameters
 *
 * no return value
 */
void dwt_configuresleep(uint16 mode, uint8 wake) {
  uint8 buf[1];

  // Add predefined sleep settings before writing the mode
  mode |= dw1000local.sleep_mode;
  dwt_write16bitoffsetreg(AON_ID, AON_WCFG_OFFSET, mode);

  buf[0] = wake;

  dwt_writetodevice(AON_ID, AON_CFG0_OFFSET, 1, buf);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_entersleepaftertx(int enable)
 *
 * @brief sets the auto TX to sleep bit. This means that after a frame
 * transmission the device will enter deep sleep mode. The dwt_setdeepsleep()
 *function
 * needs to be called before this to configure the on-wake settings
 *
 * NOTE: the IRQ line has to be low/inactive (i.e. no pending events)
 *
 * input parameters
 * @param enable - 1 to configure the device to enter deep sleep after TX, 0 -
 *disables the configuration
 *
 * output parameters
 *
 * no return value
 */
void dwt_entersleepaftertx(int enable) {
  uint32 reg = dwt_read32bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET);
  // Set the auto TX -> sleep bit
  if (enable) {
    reg |= PMSC_CTRL1_ATXSLP;
  } else {
    reg &= ~(PMSC_CTRL1_ATXSLP);
  }
  dwt_write32bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET, reg);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_spicswakeup()
 *
 * @brief wake up the device from sleep mode using the SPI read,
 * the device will wake up on chip select line going low if the line is held low
 *for at least 500us.
 * To define the length depending on the time one wants to hold
 * the chip select line low, use the following formula:
 *
 *      length (bytes) = time (s) * byte_rate (Hz)
 *
 * where fastest byte_rate is spi_rate (Hz) / 8 if the SPI is sending the bytes
 *back-to-back.
 * To save time and power, a system designer could determine byte_rate value
 *more precisely.
 *
 * NOTE: Alternatively the device can be waken up with WAKE_UP pin if configured
 *for that operation
 *
 * input parameters
 * @param buff   - this is a pointer to the dummy buffer which will be used in
 *the SPI read transaction used for the WAKE UP of the device
 * @param length - this is the length of the dummy buffer
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_spicswakeup(uint8 *buff, uint16 length) {
  if (dwt_readdevid() !=
      DWT_DEVICE_ID) // Device was in deep sleep (the first read fails)
  {
    // Need to keep chip select line low for at least 500us
    dwt_readfromdevice(
        0x0, 0x0, length,
        buff); // Do a long read to wake up the chip (hold the chip select low)

    // Need 5ms for XTAL to start and stabilise (could wait for PLL lock IRQ
    // status bit !!!)
    // NOTE: Polling of the STATUS register is not possible unless frequency is
    // < 3MHz
    deca_sleep(5);
  } else {
    return DWT_SUCCESS;
  }
  // DEBUG - check if still in sleep mode
  if (dwt_readdevid() != DWT_DEVICE_ID) {
    return DWT_ERROR;
  }

  return DWT_SUCCESS;
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_configlde()
 *
 * @brief configure LDE algorithm parameters
 *
 * input parameters
 * @param prf   -   this is the PRF index (0 or 1) 0 corresponds to 16 and 1 to
 *64 PRF
 *
 * output parameters
 *
 * no return value
 */
void _dwt_configlde(int prfIndex) {
  uint8 x = LDE_PARAM1;

  dwt_writetodevice(LDE_IF_ID, LDE_CFG1_OFFSET, 1,
                    &x); // 8-bit configuration register

  if (prfIndex) {
    dwt_write16bitoffsetreg(
        LDE_IF_ID, LDE_CFG2_OFFSET,
        (uint16)LDE_PARAM3_64); // 16-bit LDE configuration tuning register
  } else {
    dwt_write16bitoffsetreg(LDE_IF_ID, LDE_CFG2_OFFSET, (uint16)LDE_PARAM3_16);
  }
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_loaducodefromrom()
 *
 * @brief  load ucode from OTP MEMORY or ROM
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
void _dwt_loaducodefromrom(void) {
  uint8 wr_buf[2];

  // Set up clocks
  wr_buf[1] = 0x03;
  wr_buf[0] = 0x01;
  dwt_writetodevice(PMSC_ID, PMSC_CTRL0_OFFSET, 2, wr_buf);
  // Kick off the LDE load
  dwt_write16bitoffsetreg(OTP_IF_ID, OTP_CTRL,
                          OTP_CTRL_LDELOAD); // Set load LDE kick bit

  deca_sleep(1); // Allow time for code to upload (should take up to 120 us)

  // Default clocks (ENABLE_ALL_SEQ)
  _dwt_enableclocks(ENABLE_ALL_SEQ); // Enable clocks for sequencing
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_loadopsettabfromotp()
 *
 * @brief This is used to select which Operational Parameter Set table to load
 *from OTP memory
 *
 * input parameters
 * @param - opset table selection
 *                  DWT_OPSET_64LEN = 0x0 - load the operational parameter set
 *table for 64 length preamble configuration
 *                  DWT_OPSET_TIGHT = 0x1 - load the operational parameter set
 *table for tight xtal offsets (<1ppm)
 *                  DWT_OPSET_DEFLT = 0x2 - load the default operational
 *parameter set table (this is loaded from reset)
 *
 * output parameters
 *
 * no return value
 */
void dwt_loadopsettabfromotp(uint8 gtab_sel) {
  uint8 wr_buf[2];
  uint16 reg = (((gtab_sel & 0x3) << 5) | 0x1);
  // Set up clocks
  wr_buf[1] = 0x03;
  wr_buf[0] = 0x01;
  dwt_writetodevice(PMSC_ID, PMSC_CTRL0_OFFSET, 2, wr_buf);

  dwt_write16bitoffsetreg(
      OTP_IF_ID, OTP_SF,
      reg); // Set load gtab kick bit (bit0) and gtab selection bit

  // Sefault clocks (ENABLE_ALL_SEQ)
  _dwt_enableclocks(ENABLE_ALL_SEQ); // Enable clocks for sequencing
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setsmarttxpower()
 *
 * @brief This call enables or disables the smart TX power feature.
 *
 * input parameters
 * @param enable - this enables or disables the TX smart power (1 = enable, 0 =
 *disable)
 *
 * output parameters
 *
 * no return value
 */
void dwt_setsmarttxpower(int enable) {
  // Config system register
  dw1000local.sysCFGreg =
      dwt_read32bitreg(SYS_CFG_ID); // Read sysconfig register

  // Disable smart power configuration
  if (enable) {
    dw1000local.sysCFGreg &= ~(SYS_CFG_DIS_STXP);
  } else {
    dw1000local.sysCFGreg |= SYS_CFG_DIS_STXP;
  }

  dwt_write32bitreg(SYS_CFG_ID, dw1000local.sysCFGreg);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_enableautoack()
 *
 * @brief This call enables the auto-ACK feature. If the responseDelayTime
 *(parameter) is 0, the ACK will be sent a.s.a.p.
 * otherwise it will be sent with a programmed delay (in symbols), max is 255.
 * NOTE: needs to have frame filtering enabled as well
 *
 * input parameters
 * @param responseDelayTime - if non-zero the ACK is sent after this delay, max
 *is 255.
 *
 * output parameters
 *
 * no return value
 */
void dwt_enableautoack(uint8 responseDelayTime) {
  // Set auto ACK reply delay
  dwt_write16bitoffsetreg(ACK_RESP_T_ID, 0x2,
                          (responseDelayTime << 8)); // in symbols
  // Enable auto ACK
  dw1000local.sysCFGreg |= SYS_CFG_AUTOACK;
  dwt_write32bitreg(SYS_CFG_ID, dw1000local.sysCFGreg);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setdblrxbuffmode()
 *
 * @brief This call enables the double receive buffer mode
 *
 * input parameters
 * @param enable - 1 to enable, 0 to disable the double buffer mode
 *
 * output parameters
 *
 * no return value
 */
void dwt_setdblrxbuffmode(int enable) {
  if (enable) {
    // Enable double RX buffer mode
    dw1000local.sysCFGreg &= ~SYS_CFG_DIS_DRXB;
    dw1000local.dblbuffon = 1;
  } else {
    // Disable double RX buffer mode
    dw1000local.sysCFGreg |= SYS_CFG_DIS_DRXB;
    dw1000local.dblbuffon = 0;
  }

  dwt_write32bitreg(SYS_CFG_ID, dw1000local.sysCFGreg);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setautorxreenable()
 *
 * @brief This call enables the auto RX re-enable feature
 *
 * input parameters
 * @param enable - 1 to enable, 0 to disable the feature
 *
 * output parameters
 *
 * no return value
 */
void dwt_setautorxreenable(int enable) {
  uint8 byte = 0;

  if (enable) {
    // Enable auto re-enable of the receiver
    dw1000local.sysCFGreg |= SYS_CFG_RXAUTR;
  } else {
    // Disable auto re-enable of the receiver
    dw1000local.sysCFGreg &= ~SYS_CFG_RXAUTR;
  }

  byte = dw1000local.sysCFGreg >> 24;

  dwt_writetodevice(SYS_CFG_ID, 3, 1, &byte);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setrxaftertxdelay()
 *
 * @brief This sets the receiver turn on delay time after a transmission of a
 *frame
 *
 * input parameters
 * @param rxDelayTime - (20 bits) - the delay is in UWB microseconds
 *
 * output parameters
 *
 * no return value
 */
void dwt_setrxaftertxdelay(uint32 rxDelayTime) {
  uint32 val = dwt_read32bitreg(ACK_RESP_T_ID); // Read ACK_RESP_T_ID register

  val &= ~(ACK_RESP_T_W4R_TIM_MASK); // Clear the timer (19:0)

  val |= (rxDelayTime & ACK_RESP_T_W4R_TIM_MASK); // In UWB microseconds (e.g.
                                                  // turn the receiver on 20uus
                                                  // after TX)

  dwt_write32bitreg(ACK_RESP_T_ID, val);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setcallbacks()
 *
 * @brief This is the devices interrupt handler function, it will process/report
 *status events
 *
 * input parameters
 * @param txcallback - the pointer to the TX callback function
 * @param rxcallback - the pointer to the RX callback function
 *
 * output parameters
 *
 * no return value
 */
void dwt_setcallbacks(void (*txcallback)(const dwt_callback_data_t *),
                      void (*rxcallback)(const dwt_callback_data_t *)) {
  dw1000local.dwt_txcallback = txcallback;

  dw1000local.dwt_rxcallback = rxcallback;
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_checkIRQ()
 *
 * @brief This function checks if the IRQ line is active - this is used instead
 *of interrupt handler
 *
 * input parameters
 *
 * output parameters
 *
 * return value is 1 if the IRQS bit is set and 0 otherwise
 */
uint8 dwt_checkIRQ(void) {
  uint8 temp;

  dwt_readfromdevice(SYS_STATUS_ID, 0, 1, &temp);

  return (temp & 0x1);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_isr()
 *
 * @brief This is the devices interrupt handler function, it will process/report
 *status events
 * Notes:  In PC based system using (Cheetah or ARM) USB to SPI converter there
 *can be no interrupts, however we still need something
 *         to take the place of it and operate in a polled way.
 *         In an embedded system this function should be configured to launch on
 *an interrupt, then it will process the interrupt trigger event and
 *         call a TX or RX call-back function depending on whether the event is
 *a TX or RX event.
 *         The TX call-back will be called when a frame has been sent and the RX
 *call-back when a frame has been received.
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
void dwt_isr(void) // Assume interrupt can supply context
{
  uint32 status = 0;
  uint32 clear = 0; // Will clear any events seen

  dw1000local.cdata.event = 0;
  dw1000local.cdata.dblbuff = dw1000local.dblbuffon;

  status = dw1000local.cdata.status =
      dwt_read32bitreg(SYS_STATUS_ID); // Read status register low 32bits

  // NOTES:
  // 1. TX Event - if DWT_INT_TFRS is enabled, then when the frame has completed
  // transmission the interrupt will be triggered.
  //   The status register will have the TXFRS bit set. This function will clear
  //   the tx event and call the dwt_txcallback function.
  //
  // 2. RX Event - if DWT_INT_RFCG is enabled, then when a frame with good CRC
  // has been received the interrupt will be triggered.
  //   The status register will have the RXFCG bit set. This function will clear
  //   the rx event and call the dwt_rxcallback function.
  //
  // 2.a. RX Event - This is same as 2. above except this time the received
  // frame has ACK request bit set in the header (AAT bit will be set).
  //     This function will clear the rx event and call the dwt_rxcallback
  //     function, notifying the application that ACK req is set.
  //     If using auto-ACK, the AAT indicates that ACK frame transmission is in
  //     progress. Once the ACK has been sent the TXFRS bit will be set and TX
  //     event triggered.
  //     If the auto-ACK is not enabled, the application can format/configure
  //     and start transmission of its own ACK frame.
  //

  // Fix for bug 622 - LDE done flag gets latched on a bad frame
  if ((status & SYS_STATUS_LDEDONE) && (dw1000local.dblbuffon == 0)) {
    if ((status &
         (SYS_STATUS_LDEDONE | SYS_STATUS_RXPHD | SYS_STATUS_RXSFDD)) !=
        (SYS_STATUS_LDEDONE | SYS_STATUS_RXPHD | SYS_STATUS_RXSFDD)) {

      // Got LDE done but other flags SFD and PHR are clear - this is a bad
      // frame - reset the transceiver
      dwt_forcetrxoff(); // this will clear all events

      dwt_rxreset();
      // Leave any TX events for processing (e.g. if we TX a frame, and then
      // enable RX,
      // We can get into here before the TX frame done has been processed, when
      // we are polling (i.e. slow to process the TX)
      status &= SYS_STATUS_ALL_TX;
      // Re-enable the receiver - if auto RX re-enable set
      if (dw1000local.sysCFGreg & SYS_CFG_RXAUTR) {
        dwt_write16bitoffsetreg(SYS_CTRL_ID, 0, (uint16)SYS_CTRL_RXENAB);
      } else {
        dw1000local.cdata.event = DWT_SIG_RX_ERROR;

        if (dw1000local.dwt_rxcallback != NULL)
          dw1000local.dwt_rxcallback(&dw1000local.cdata);
      }
    }
  }

  //
  // 1st check for RX frame received or RX timeout and if so call the rx
  // callback function
  //
  if (status & SYS_STATUS_RXFCG) // Receiver FCS Good
  {
    if (status & SYS_STATUS_LDEDONE) // LDE done/finished
    {
      // Bug 634 - overrun overwrites the frame info data... so both frames
      // should be discarded
      // Read frame info and other registers and check for overflow again
      // If overflow set then discard both frames...

      uint16 len = 0;

      if (status & SYS_STATUS_RXOVRR) // NOTE when overrun both HS and RS
                                      // pointers point to the same buffer
      {
        // When the overrun happens the frame info data of the buffer A (which
        // contains the older frame e.g. seq. num = x)
        // will be corrupted with the latest frame (seq. num = x + 2) data, both
        // the host and IC are pointing to buffer A
        // We are going to discard this frame - turn off transceiver and reset
        // receiver
        dwt_forcetrxoff();

        dwt_rxreset();

        if (dw1000local.sysCFGreg & SYS_CFG_RXAUTR) // Re-enable of RX is ON,
                                                    // then re-enable here
                                                    // (ignore error)
        {
          dwt_write16bitoffsetreg(SYS_CTRL_ID, 0, (uint16)SYS_CTRL_RXENAB);
        } else // The RX will be re-enabled by the application, report an error
        {
          dw1000local.cdata.event = DWT_SIG_RX_ERROR;

          if (dw1000local.dwt_rxcallback != NULL) {
            dw1000local.dwt_rxcallback(&dw1000local.cdata);
          }
        }

        return;
      } else // No overrun condition - proceed to process the frame
      {

        len = dwt_read16bitoffsetreg(RX_FINFO_ID, 0) & 0x3FF;
        dwt_readfromdevice(RX_BUFFER_ID, 0, 2, dw1000local.cdata.fctrl);
        if (dw1000local.longFrames == 0) {
          len &= 0x7F;
        }

        // Standard frame length up to 127, extended frame length up to 1023
        // bytes
        dw1000local.cdata.datalength = len;

        // Bug 627 workaround - clear the AAT bit if the ACK request bit in the
        // FC is not set
        if ((status & SYS_STATUS_AAT) // AAT bit is set (ACK has been requested)
            && (((dw1000local.cdata.fctrl[0] & 0x20) == 0) ||
                (dw1000local.cdata.fctrl[0] ==
                 0x02)) // But the data frame has it clear or it is an ACK frame
            ) {
          clear |= SYS_STATUS_AAT;
          dw1000local.cdata.aatset = 0; // ACK request is not set
          dw1000local.wait4resp = 0;
        } else // The AAT is correctly set for a frame that requested the ACK
        {
          dw1000local.cdata.aatset =
              (status & SYS_STATUS_AAT); // check if ACK request is set
        }

        dw1000local.cdata.event = DWT_SIG_RX_OKAY;

        if (dw1000local.dblbuffon == 0) // If no double buffering
        {
          // Clear all receive status bits (as we are finished with this receive
          // event)
          clear |= status & SYS_STATUS_ALL_RX_GOOD;
          dwt_write32bitreg(
              SYS_STATUS_ID,
              clear); // Write status register to clear event bits we have seen

          // NOTE: clear the event which caused interrupt means once the RX is
          // enabled or TX is started
          // New events can trigger and give rise to new interrupts
          // Call the RX call-back function to process the RX event
          if (dw1000local.dwt_rxcallback != NULL) {
            dw1000local.dwt_rxcallback(&dw1000local.cdata);
          }
        } else // Double buffer
        {
          uint8 buff;
          uint8 hsrb = 0x01;

          // Need to make sure that the host/IC buffer pointers are aligned
          // before starting RX
          // Read again because the status could have changed since the
          // interrupt was triggered
          dwt_readfromdevice(SYS_STATUS_ID, 3, 1, &buff);
          // If ICRBP is equal to HSRBP this means we've received another frame
          // (both buffers have frames)
          if ((buff &
               (SYS_STATUS_ICRBP >> 24)) == // IC side Receive Buffer Pointer
              ((buff & (SYS_STATUS_HSRBP >> 24))
               << 1)) // Host Side Receive Buffer Pointer
          {
            // Clear all receive status bits (as we are finished with this
            // receive event)
            clear |= status & SYS_STATUS_ALL_DBLBUFF;
            dwt_write32bitreg(SYS_STATUS_ID, clear); // Write status register to
                                                     // clear event bits we have
                                                     // seen
          }
          // If they are not aligned then there is a new frame in the other
          // buffer, so we just need to toggle...

          if ((dw1000local.sysCFGreg & SYS_CFG_RXAUTR) ==
              0) // Double buffer is on but no auto RX re-enable RX
          {
            dwt_write16bitoffsetreg(SYS_CTRL_ID, 0, (uint16)SYS_CTRL_RXENAB);
          }

          // Call the RX call-back function to process the RX event
          if (dw1000local.dwt_rxcallback != NULL) {
            dw1000local.dwt_rxcallback(&dw1000local.cdata);
          }
          // If overrun, then reset the receiver - RX bug, when overruns cannot
          // guarantee the last frame's data was not corrupted
          // If no overrun all is good, toggle the pointer
          if (dwt_checkoverrun() == 0) {
            // Toggle the host side Receive Buffer Pointer by writing one to the
            // register
            dwt_writetodevice(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET, 1,
                              &hsrb); // We need to swap RX buffer status reg
                                      // (write one to toggle internally)
          } else {
            // The call-back has completed, but the overrun has been set, before
            // we toggled, this means two new frames have arrived (one in the
            // other buffer) and the 2nd's PHR good set the overrun flag
            // Due to a receiver bug, which cannot guarantee the last frame's
            // data was not corrupted need to reset receiver and discard any new
            // data
            dwt_forcetrxoff();

            dwt_rxreset();

            if (dw1000local.sysCFGreg &
                SYS_CFG_RXAUTR) // Re-enable of RX is ON, then re-enable here
            {
              dwt_write16bitoffsetreg(SYS_CTRL_ID, 0, (uint16)SYS_CTRL_RXENAB);
            }
          }
        } // end of else double buffer

      }  // end of no overrun
    }    // If LDE_DONE is set (this means we have both SYS_STATUS_RXFCG and
         // SYS_STATUS_LDEDONE)
    else // No LDE_DONE ?
    {
      // printf("NO LDE done or LDE error\n");
      if (!(dw1000local.sysCFGreg & SYS_CFG_RXAUTR)) {
        dwt_forcetrxoff();
      }
      dwt_rxreset(); // Reset the RX
      dw1000local.wait4resp = 0;
      dw1000local.cdata.event = DWT_SIG_RX_ERROR;
      if (dw1000local.dwt_rxcallback != NULL) {
        dw1000local.dwt_rxcallback(&dw1000local.cdata);
      }
    }
  } // end if CRC is good
  else
      //
      // Check for TX frame sent event and signal to upper layer.
      //
      if (status & SYS_STATUS_TXFRS) // Transmit Frame Sent
  {
    clear |= SYS_STATUS_ALL_TX; // clear TX event bits
    dwt_write32bitreg(
        SYS_STATUS_ID,
        clear); // Write status register to clear event bits we have seen
    // NOTE: clear the event which caused interrupt means once the RX is enabled
    // or TX is started
    // New events can trigger and give rise to new interrupts
    if (dw1000local.cdata.aatset) {
      dw1000local.cdata.aatset = 0;   // The ACK has been sent
      if (dw1000local.dblbuffon == 0) // If not double buffered
      {
        if (dw1000local.wait4resp) // wait4response was set with the last TX
                                   // start command
        {
          // If using wait4response and the ACK has been sent as the response
          // requested it
          // the receiver will be re-enabled, so issue a TRXOFF command to
          // disable and prevent any
          // unexpected interrupts
          dwt_forcetrxoff();
        }
      }
    }

    dw1000local.cdata.event = DWT_SIG_TX_DONE; // Signal TX completed

    // Call the TX call-back function to process the TX event
    if (dw1000local.dwt_txcallback != NULL) {
      dw1000local.dwt_txcallback(&dw1000local.cdata);
    }

  } else if (status & SYS_STATUS_RXRFTO) // Receiver Frame Wait timeout
  {
    clear |= status & SYS_STATUS_RXRFTO;
    dwt_write32bitreg(
        SYS_STATUS_ID,
        clear); // Write status register to clear event bits we have seen
    dw1000local.cdata.event = DWT_SIG_RX_TIMEOUT;
    if (dw1000local.dwt_rxcallback != NULL) {
      dw1000local.dwt_rxcallback(&dw1000local.cdata);
    }
    dw1000local.wait4resp = 0;

  } else if (status & SYS_STATUS_ALL_RX_ERR) // Catches all other error events
  {
    clear |= status & SYS_STATUS_ALL_RX_ERR;
    dwt_write32bitreg(
        SYS_STATUS_ID,
        clear); // Write status register to clear event bits we have seen

    dw1000local.wait4resp = 0;
    // NOTE: clear the event which caused interrupt means once the RX is enabled
    // or TX is started
    // New events can trigger and give rise to new interrupts

    // Fix for bug 622 - LDE done flag gets latched on a bad frame / reset
    // receiver
    if (!(dw1000local.sysCFGreg & SYS_CFG_RXAUTR)) {
      dwt_forcetrxoff(); // This will clear all events
    }
    dwt_rxreset(); // Reset the RX
    // End of fix for bug 622 - LDE done flag gets latched on a bad frame

    if (status & SYS_STATUS_RXPHE) {
      dw1000local.cdata.event = DWT_SIG_RX_PHR_ERROR;
    } else if (status & SYS_STATUS_RXFCE) {
      dw1000local.cdata.event = DWT_SIG_RX_ERROR;
    } else if (status & SYS_STATUS_RXRFSL) {
      dw1000local.cdata.event = DWT_SIG_RX_SYNCLOSS;
    } else if (status & SYS_STATUS_RXSFDTO) {
      dw1000local.cdata.event = DWT_SIG_RX_SFDTIMEOUT;
    } else if (status & SYS_STATUS_RXPTO) {
      dw1000local.cdata.event = DWT_SIG_RX_PTOTIMEOUT;
    } else {
      dw1000local.cdata.event = DWT_SIG_RX_ERROR;
    }
    if (dw1000local.dwt_rxcallback != NULL) {
      dw1000local.dwt_rxcallback(&dw1000local.cdata);
    }
    status &= SYS_STATUS_ALL_TX;
  }
} // end dwt_isr()

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setleds()
 *
 * @brief This is used to set up Tx/Rx GPIOs which could be used to control LEDs
 * Note: not completely IC dependent, also needs board with LEDS fitted on right
 *I/O lines
 *       this function enables GPIOs 2 and 3 which are connected to LED3 and
 *LED4 on EVB1000
 *
 * input parameters
 * @param test - if 1 the LEDs will be enabled, if 0 the LED control is
 *disabled.
 *             - if value is 2 the LEDs will flash once after enable.
 *
 * output parameters
 *
 * no return value
 */
void dwt_setleds(uint8 test) {
  uint8 buf[2];

  if (test & 0x1) {
    // Set up MFIO for LED output
    dwt_readfromdevice(GPIO_CTRL_ID, 0x00, 2, buf);
    buf[1] &= ~0x3C; // clear the bits
    buf[1] |= 0x14;
    dwt_writetodevice(GPIO_CTRL_ID, 0x01, 1, &buf[1]);

    // Enable LP Oscillator to run from counter, turn on debounce clock
    dwt_readfromdevice(PMSC_ID, 0x02, 1, buf);
    buf[0] |= 0x84; //
    dwt_writetodevice(PMSC_ID, 0x02, 1, buf);

    // Enable LEDs to blink
    buf[0] = 0x10; // Blink period.
    buf[1] = 0x01; // Enable blink counter
    dwt_writetodevice(PMSC_ID, PMSC_LEDC_OFFSET, 2, buf);

  } else if ((test & 0x1) == 0) {
    // Clear the GPIO bits that are used for LED control
    dwt_readfromdevice(GPIO_CTRL_ID, 0x00, 2, buf);
    buf[1] &= ~(0x14);
    dwt_writetodevice(GPIO_CTRL_ID, 0x00, 2, buf);
  }

  // Test LEDs
  if (test & 0x2) {
    buf[0] = 0x0f; // Fire a LED blink trigger
    dwt_writetodevice(PMSC_ID, 0x2a, 1, buf);
    buf[0] = 0x00; // Clear forced trigger bits
    dwt_writetodevice(PMSC_ID, 0x2a, 1, buf);
  }

} // end _dwt_enableleds()

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_enableclocks()
 *
 * @brief function to enable/disable clocks to particular digital blocks/system
 *
 * input parameters
 * @param clocks - set of clocks to enable/disable
 *
 * output parameters none
 *
 * no return value
 */
void _dwt_enableclocks(int clocks) {
  uint8 reg[2];

  dwt_readfromdevice(PMSC_ID, PMSC_CTRL0_OFFSET, 2, reg);
  switch (clocks) {
  case ENABLE_ALL_SEQ: {
    reg[0] = 0x00;
    reg[1] = reg[1] & 0xfe;
  } break;
  case FORCE_SYS_XTI: {
    // System and RX
    reg[0] = 0x01 | (reg[0] & 0xfc);
  } break;
  case FORCE_SYS_PLL: {
    // System
    reg[0] = 0x02 | (reg[0] & 0xfc);
  } break;
  case READ_ACC_ON: {
    reg[0] = 0x48 | (reg[0] & 0xb3);
    reg[1] = 0x80 | reg[1];
  } break;
  case READ_ACC_OFF: {
    reg[0] = reg[0] & 0xb3;
    reg[1] = 0x7f & reg[1];

  } break;
  case FORCE_OTP_ON: {
    reg[1] = 0x02 | reg[1];
  } break;
  case FORCE_OTP_OFF: {
    reg[1] = reg[1] & 0xfd;
  } break;
  case FORCE_TX_PLL: {
    reg[0] = 0x20 | (reg[0] & 0xcf);
  } break;
  default:
    break;
  }

  // Need to write lower byte separately before setting the higher byte(s)
  dwt_writetodevice(PMSC_ID, PMSC_CTRL0_OFFSET, 1, &reg[0]);
  dwt_writetodevice(PMSC_ID, 0x1, 1, &reg[1]);

} // end _dwt_enableclocks()

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_disablesequencing()
 *
 * @brief This function disables the TX blocks sequencing, it disables PMSC
 *control of RF blocks, system clock is also set to XTAL
 *
 * input parameters none
 *
 * output parameters none
 *
 * no return value
 */
void _dwt_disablesequencing(void) // Disable sequencing and go to state "INIT"
{
  _dwt_enableclocks(FORCE_SYS_XTI); // Set system clock to XTI

  dwt_write16bitoffsetreg(
      PMSC_ID, PMSC_CTRL1_OFFSET,
      PMSC_CTRL1_PKTSEQ_DISABLE); // Disable PMSC ctrl of RF and RX clk blocks
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setdelayedtrxtime()
 *
 * @brief This API function configures the delayed transmit time or the delayed
 *RX on time
 *
 * input parameters
 * @param starttime - the TX/RX start time (the 32 bits should be the high 32
 *bits of the system time at which to send the message,
 * or at which to turn on the receiver)
 *
 * output parameters none
 *
 * no return value
 */
void dwt_setdelayedtrxtime(uint32 starttime) {
  dwt_write32bitoffsetreg(DX_TIME_ID, 1, starttime);

} // end dwt_setdelayedtrxtime()

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_starttx()
 *
 * @brief This call initiates the transmission, input parameter indicates which
 *TX mode is used see below
 *
 * input parameters:
 * @param mode - if 0 immediate TX (no response expected)
 *               if 1 delayed TX (no response expected)
 *               if 2 immediate TX (response expected - so the receiver will be
 *automatically turned on after TX is done)
 *               if 3 delayed TX (response expected - so the receiver will be
 *automatically turned on after TX is done)
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error (e.g. a delayed
 *transmission will fail if the delayed time has passed)
 */
int dwt_starttx(uint8 mode) {
  int retval = DWT_SUCCESS;
  uint8 temp = 0x00;
  uint16 checkTxOK = 0;

  if (mode & DWT_RESPONSE_EXPECTED) {
    temp = (uint8)SYS_CTRL_WAIT4RESP; // Set wait4response bit
    dwt_writetodevice(SYS_CTRL_ID, 0, 1, &temp);
    dw1000local.wait4resp = 1;
  }

  if (mode & DWT_START_TX_DELAYED) {
    // uint32 status ;

    // Both SYS_CTRL_TXSTRT and SYS_CTRL_TXDLYS to correctly enable TX
    temp |= (uint8)(SYS_CTRL_TXDLYS | SYS_CTRL_TXSTRT);
    dwt_writetodevice(SYS_CTRL_ID, 0, 1, &temp);
    checkTxOK = dwt_read16bitoffsetreg(SYS_STATUS_ID, 3);
    // status = dwt_read32bitreg(SYS_STATUS_ID) ; // Read status register
    if ((checkTxOK & SYS_STATUS_TXERR) == 0) // Transmit Delayed Send set over
                                             // Half a Period away or Power Up
                                             // error (there is enough time to
                                             // send but not to power up
                                             // individual blocks).
    {
      // printf("tx delayed \n");
      retval = DWT_SUCCESS; // All okay
    } else {
      // I am taking DSHP set to Indicate that the TXDLYS was set too late for
      // the specified DX_TIME.
      // Remedial Action - (a) cancel delayed send
      temp =
          (uint8)SYS_CTRL_TRXOFF; // This assumes the bit is in the lowest byte
      dwt_writetodevice(SYS_CTRL_ID, 0, 1, &temp);
      // Note event Delayed TX Time too Late
      // Could fall through to start a normal send (below) just sending
      // late.....
      // ... instead return and assume return value of 1 will be used to detect
      // and recover from the issue.

      // Clear the "auto TX to sleep" bit
      dwt_entersleepaftertx(0);
      dw1000local.wait4resp = 0;
      retval = DWT_ERROR; // Failed !
    }
  } else {
    temp |= (uint8)SYS_CTRL_TXSTRT;
    dwt_writetodevice(SYS_CTRL_ID, 0, 1, &temp);
  }

  return retval;

} // end dwt_starttx()

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_checkoverrun()
 *
 * @brief This is used to check if the overrun condition is set in DW1000
 *
 * input parameters
 *
 * output parameters
 *
 * returns 1 if the RXOVERR bit is set, else 0
 */
int dwt_checkoverrun(void) {
  return ((dwt_read16bitoffsetreg(SYS_STATUS_ID, 2) &
           (SYS_STATUS_RXOVRR >> 16)) == (SYS_STATUS_RXOVRR >> 16));
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_forcetrxoff()
 *
 * @brief This is used to turn off the transceiver
 *
 * input parameters
 *
 * output parameters
 *
 * no return value
 */
void dwt_forcetrxoff(void) {
  decaIrqStatus_t stat;
  uint8 temp;
  uint32 mask;

  temp = (uint8)SYS_CTRL_TRXOFF; // This assumes the bit is in the lowest byte

  mask = dwt_read32bitreg(SYS_MASK_ID); // Read set interrupt mask

  // Need to beware of interrupts occurring in the middle of following read
  // modify write cycle
  // We can disable the radio, but before the status is cleared an interrupt can
  // be set (e.g. the
  // event has just happened before the radio was disabled)
  // thus we need to disable interrupt during this operation
  stat = decamutexon();

  dwt_write32bitreg(
      SYS_MASK_ID,
      0); // Clear interrupt mask - so we don't get any unwanted events

  dwt_writetodevice(SYS_CTRL_ID, 0, 1, &temp); // Disable the radio

  // Forcing Transceiver off - so we do not want to see any new events that may
  // have happened
  dwt_write32bitreg(SYS_STATUS_ID, (SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_ERR |
                                    SYS_STATUS_ALL_RX_GOOD));

  dwt_syncrxbufptrs();

  dwt_write32bitreg(SYS_MASK_ID, mask); // Set interrupt mask to what it was

  // Enable/restore interrupts again...
  decamutexoff(stat);
  dw1000local.wait4resp = 0;

} // end deviceforcetrxoff()

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_syncrxbufptrs()
 *
 * @brief this function synchronizes rx buffer pointers
 * need to make sure that the host/IC buffer pointers are aligned before
 *starting RX
 *
 * input parameters:
 *
 * output parameters
 *
 * no return value
 */
void dwt_syncrxbufptrs(void) {
  uint8 buff;
  // Need to make sure that the host/IC buffer pointers are aligned before
  // starting RX
  dwt_readfromdevice(SYS_STATUS_ID, 3, 1, &buff);

  if ((buff & (SYS_STATUS_ICRBP >> 24)) != // IC side Receive Buffer Pointer
      ((buff & (SYS_STATUS_HSRBP >> 24))
       << 1)) // Host Side Receive Buffer Pointer
  {
    uint8 hsrb = 0x01;
    dwt_writetodevice(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET, 1,
                      &hsrb); // We need to swap RX buffer status reg (write one
                              // to toggle internally)
  }
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setrxmode()
 *
 *  @brief enable different RX modes, e.g.:
 *   a) "snooze" mode, the receiver only listens periodically for preamble
 *   b) the RX PPDM "sniff" mode - receiver cycles through ON/OFF periods
 *
 * input parameters:
 * @param mode      - DWT_RX_NORMAL = 0x0
 *                    DWT_RX_SNIFF  = 0x1      enable the rx PPDM "sniff" mode
 * @param rxON      - SNIFF mode ON period in PACs
 * @param rxOFF     - SNIFF mode OFF period in us (actually in 1.0256 micro
 *second intervals)
 *
 * output parameters
 *
 * no return value
 */
void dwt_setrxmode(int mode, uint8 rxON, uint8 rxOFF) {
  uint16 reg16 = RX_SNIFF_MASK & ((rxOFF << 8) | rxON);

  if (mode & DWT_RX_SNIFF) {
    // PPM_OFF 15:8, PPM_ON 3:0
    dwt_write16bitoffsetreg(RX_SNIFF_ID, 0x00, reg16); // Enable
  } else {
    dwt_write16bitoffsetreg(RX_SNIFF_ID, 0x00, 0x0000); // Disable
  }
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_rxenable()
 *
 * @brief This call turns on the receiver, can be immediate or delayed.
 * The receiver will stay turned on, listening to any messages until
 * it either receives a good frame, an error (CRC, PHY header, Reed Solomon) or
 *it times out (SFD, Preamble or Frame).
 *
 * input parameters
 * @param delayed - TRUE the receiver is turned on after some delay (as
 *programmed with dwt_setdelayedtime())
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error (e.g. a delayed
 *receive enable will be too far in the future if delayed time has passed (if
 *delayed time is > 8s from now))
 */
int dwt_rxenable(int delayed) {
  uint16 temp;
  uint8 temp1 = 0;
  dwt_syncrxbufptrs();

  temp = (uint16)SYS_CTRL_RXENAB;

  if (delayed) {
    temp |= (uint16)SYS_CTRL_RXDLYE;
  }

  dwt_write16bitoffsetreg(SYS_CTRL_ID, 0, temp);

  if (delayed) // Check for errors
  {
    // uint32 status1 = dwt_read32bitreg(SYS_STATUS_ID) ; // Read status
    // register

    dwt_readfromdevice(SYS_STATUS_ID, 3, 1, &temp1);

    if (temp1 & (SYS_STATUS_HPDWARN >>
                 24)) // If delay has not passed do delayed else immediate RX on
    {
      dwt_forcetrxoff(); // Turn the delayed receive off, and do immediate
                         // receive, return warning indication
      temp = (uint16)SYS_CTRL_RXENAB; // Clear the delay bit
      dwt_write16bitoffsetreg(SYS_CTRL_ID, 0, temp);
      return DWT_ERROR;
    }
  }

  return DWT_SUCCESS;
} // end dwt_rxenable()

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setrxtimeout()
 *
 * @brief This call enables RX timeout (SY_STAT_RFTO event)
 *
 * input parameters
 * @param time - how long the receiver remains on from the RX enable command
 *               The time parameter used here is in 1.0256 us (512/499.2MHz)
 *units
 *               If set to 0 the timeout is disabled.
 *
 * output parameters
 *
 * no return value
 */
void dwt_setrxtimeout(uint16 time) {
  uint8 temp;

  dwt_readfromdevice(SYS_CFG_ID, 3, 1, &temp); // Read register

  if (time > 0) {
    dwt_write16bitoffsetreg(RX_FWTO_ID, 0x0, time);

    temp |= (uint8)(SYS_CFG_RXWTOE >> 24);
    // OR in 32bit value (1 bit set), I know this is in high byte.
    dw1000local.sysCFGreg |= SYS_CFG_RXWTOE;

    dwt_writetodevice(SYS_CFG_ID, 3, 1, &temp);
  } else {
    temp &= ~((uint8)(SYS_CFG_RXWTOE >> 24));
    // AND in inverted 32bit value (1 bit clear), I know this is in high byte.
    dw1000local.sysCFGreg &= ~(SYS_CFG_RXWTOE);

    dwt_writetodevice(SYS_CFG_ID, 3, 1, &temp);

    // dwt_write16bitoffsetreg(RX_FWTO_ID,0,0) ; // Clearing the time is not
    // needed
  }

} // end dwt_setrxtimeout()

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_setpreambledetecttimeout()
 *
 * @brief This call enables preamble timeout (SY_STAT_RXPTO event)
 *
 * input parameters
 * @param  timeout - in PACs
 *
 * output parameters
 *
 * no return value
 */
void dwt_setpreambledetecttimeout(uint16 timeout) {
  dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_PRETOC_OFFSET, timeout);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn void dwt_setinterrupt()
 *
 * @brief This function enables the specified events to trigger an interrupt.
 * The following events can be enabled:
 * DWT_INT_TFRS         0x00000080          // frame sent
 * DWT_INT_RFCG         0x00004000          // frame received with good CRC
 * DWT_INT_RPHE         0x00001000          // receiver PHY header error
 * DWT_INT_RFCE         0x00008000          // receiver CRC error
 * DWT_INT_RFSL         0x00010000          // receiver sync loss error
 * DWT_INT_RFTO         0x00020000          // frame wait timeout
 * DWT_INT_RXPTO        0x00200000          // preamble detect timeout
 * DWT_INT_SFDT         0x04000000          // SFD timeout
 * DWT_INT_ARFE         0x20000000          // frame rejected (due to frame
 *filtering configuration)
 *
 *
 * input parameters:
 * @param bitmask - sets the events which will generate interrupt
 * @param enable - if set the interrupts are enabled else they are cleared
 *
 * output parameters
 *
 * no return value
 */
void dwt_setinterrupt(uint32 bitmask, uint8 enable) {
  decaIrqStatus_t stat;
  uint32 mask;

  // Need to beware of interrupts occurring in the middle of following read
  // modify write cycle
  stat = decamutexon();

  mask = dwt_read32bitreg(SYS_MASK_ID); // Read register

  if (enable) {
    mask |= bitmask;
  } else {
    mask &= ~bitmask; // Clear the bit
  }
  dwt_write32bitreg(SYS_MASK_ID, mask); // New value

  decamutexoff(stat);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_configeventcounters()
 *
 * @brief This is used to enable/disable the event counter in the IC
 *
 * input parameters
 * @param - enable - 1 enables (and reset), 0 disables the event counters
 * output parameters
 *
 * no return value
 */
void dwt_configeventcounters(int enable) {
  uint8 temp = 0x0; // disable
  // Need to clear and disable, can't just clear
  temp = (uint8)(EVC_CLR); // Clear and disable
  dwt_writetodevice(DIG_DIAG_ID, EVC_CTRL_OFFSET, 1, &temp);

  if (enable) {
    temp = (uint8)(EVC_EN); // Enable
    dwt_writetodevice(DIG_DIAG_ID, EVC_CTRL_OFFSET, 1, &temp);
  }
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readeventcounters()
 *
 * @brief This is used to read the event counters in the IC
 *
 * input parameters
 * @param counters - pointer to the dwt_deviceentcnts_t structure which will
 *hold the read data
 *
 * output parameters
 *
 * no return value
 */
void dwt_readeventcounters(dwt_deviceentcnts_t *counters) {
  uint32 temp;

  temp = dwt_read32bitoffsetreg(
      DIG_DIAG_ID, EVC_PHE_OFFSET); // Read sync loss (31-16), PHE (15-0)
  counters->PHE = temp & 0xFFF;
  counters->RSL = (temp >> 16) & 0xFFF;

  temp = dwt_read32bitoffsetreg(
      DIG_DIAG_ID, EVC_FCG_OFFSET); // Read CRC bad (31-16), CRC good (15-0)
  counters->CRCG = temp & 0xFFF;
  counters->CRCB = (temp >> 16) & 0xFFF;

  temp = dwt_read32bitoffsetreg(
      DIG_DIAG_ID, EVC_FFR_OFFSET); // Overruns (31-16), address errors (15-0)
  counters->ARFE = temp & 0xFFF;
  counters->OVER = (temp >> 16) & 0xFFF;

  temp = dwt_read32bitoffsetreg(
      DIG_DIAG_ID, EVC_STO_OFFSET); // Read PTO (31-16), SFDTO (15-0)
  counters->PTO = (temp >> 16) & 0xFFF;
  counters->SFDTO = temp & 0xFFF;

  temp = dwt_read32bitoffsetreg(
      DIG_DIAG_ID, EVC_FWTO_OFFSET); // Read RX TO (31-16), TXFRAME (15-0)
  counters->TXF = (temp >> 16) & 0xFFF;
  counters->RTO = temp & 0xFFF;

  temp = dwt_read32bitoffsetreg(
      DIG_DIAG_ID, EVC_HPW_OFFSET); // Read half period warning events
  counters->HPW = temp & 0xFFF;
  counters->TXW = (temp >> 16) & 0xFFF; // Power-up warning events
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_rxreset()
 *
 * @brief this function resets the receiver of the DW1000
 *
 * input parameters:
 *
 * output parameters
 *
 * no return value
 */
void dwt_rxreset(void) {
  uint8 resetrx = 0xe0;
  // Set RX reset
  dwt_writetodevice(PMSC_ID, 0x3, 1, &resetrx);

  resetrx = 0xf0; // Clear RX reset
  dwt_writetodevice(PMSC_ID, 0x3, 1, &resetrx);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_softreset()
 *
 * @brief this function resets the DW1000
 *
 * input parameters:
 *
 * output parameters
 *
 * no return value
 */
void dwt_softreset(void) {
  uint8 temp[1] = {0};

  _dwt_disablesequencing();
  //_dwt_enableclocks(FORCE_SYS_XTI); // Set system clock to XTI

  // Clear any AON auto download bits (as reset will trigger AON download)
  dwt_write16bitoffsetreg(AON_ID, AON_WCFG_OFFSET, 0x0);
  // Clear the wake-up configuration
  dwt_writetodevice(AON_ID, AON_CFG0_OFFSET, 1, temp);
  // Upload the new configuration
  _dwt_aonarrayupload();

  // Reset HIF, TX, RX and PMSC
  dwt_readfromdevice(PMSC_ID, 0x3, 1, temp);

  temp[0] &= 0x0F;
  dwt_writetodevice(PMSC_ID, 0x3, 1, &temp[0]);

  // DW1000 needs a 10us sleep to let clk PLL lock after reset - the PLL will
  // automatically lock after the reset
  // Could also have polled the PLL lock flag, but then the SPI needs to be <
  // 3MHz !! So a simple delay is easier
  deca_sleep(1);

  temp[0] |= 0xF0;
  dwt_writetodevice(PMSC_ID, 0x3, 1, &temp[0]);

  dw1000local.wait4resp = 0;
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_xtaltrim()
 *
 * @brief This is used adjust the crystal frequency
 *
 * input parameters:
 * @param   value - crystal trim value (in range 0x0 to 0x1F) 31 steps (~1.5ppm
 *per step)
 *
 * @output
 *
 * no return value
 */
void dwt_xtaltrim(uint8 value) {
  uint8 write_buf;

  dwt_readfromdevice(FS_CTRL_ID, FS_XTALT_OFFSET, 1, &write_buf);

  write_buf &= ~FS_XTALT_MASK;

  write_buf |= (FS_XTALT_MASK & value); // We should not change high bits, cause
                                        // it will cause malfunction

  dwt_writetodevice(FS_CTRL_ID, FS_XTALT_OFFSET, 1, &write_buf);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_configcwmode()
 *
 * @brief this function sets the DW1000 to transmit cw signal at specific
 *channel frequency
 *
 * input parameters:
 * @param chan - specifies the operating channel (e.g. 1, 2, 3, 4, 5, 6 or 7)
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_configcwmode(uint8 chan) {
  uint8 write_buf[1];
#ifdef DWT_API_ERROR_CHECK
  if ((chan < 1) || (chan > 7) || (6 == chan)) {
    return DWT_ERROR; // validate channel number parameter
  }
#endif

  //
  // Disable TX/RX RF block sequencing (needed for cw frame mode)
  //
  _dwt_disablesequencing();

  // Config RF pll (for a given channel)
  // Configure PLL2/RF PLL block CFG
  dwt_writetodevice(FS_CTRL_ID, FS_PLLCFG_OFFSET, 5,
                    &pll2_config[chan_idx[chan]][0]);
  // PLL wont be enabled until a TX/RX enable is issued later on
  // Configure RF TX blocks (for specified channel and prf)
  // Config RF TX control
  dwt_write32bitoffsetreg(RF_CONF_ID, RF_TXCTRL_OFFSET,
                          tx_config[chan_idx[chan]]);

  //
  // Enable RF PLL
  //
  dwt_write32bitreg(RF_CONF_ID,
                    RF_CONF_TXPLLPOWEN_MASK); // Enable LDO and RF PLL blocks
  dwt_write32bitreg(RF_CONF_ID,
                    RF_CONF_TXALLEN_MASK); // Enable the rest of TX blocks

  //
  // Configure TX clocks
  //
  write_buf[0] = 0x22;
  dwt_writetodevice(PMSC_ID, PMSC_CTRL0_OFFSET, 1, write_buf);
  write_buf[0] = 0x07;
  dwt_writetodevice(PMSC_ID, 0x1, 1, write_buf);

  // Disable fine grain TX seq
  dwt_write16bitoffsetreg(PMSC_ID, PMSC_TXFINESEQ_OFFSET,
                          PMSC_TXFINESEQ_DIS_MASK);

  write_buf[0] = TC_PGTEST_CW;

  // Configure CW mode
  dwt_writetodevice(TX_CAL_ID, TC_PGTEST_OFFSET, TC_PGTEST_LEN, write_buf);

  return DWT_SUCCESS;
}

/*!
*------------------------------------------------------------------------------------------------------------------
 * @fn dwt_configcontinuousframemode()
 *
 * @brief this function sets the DW1000 to continuous tx frame mode for
*regulatory approvals testing.
 *
 * input parameters:
 * @param framerepetitionrate - This is a 32-bit value that is used to set the
*interval between transmissions.
*  The minimum value is 4. The units are approximately 8 ns. (or more precisely
*512/(499.2e6*128) seconds)).
 *
 * output parameters
 *
 * no return value
 */
void dwt_configcontinuousframemode(uint32 framerepetitionrate) {
  uint8 write_buf[4];

  //
  // Disable TX/RX RF block sequencing (needed for continuous frame mode)
  //
  _dwt_disablesequencing();

  //
  // Enable RF PLL and TX blocks
  //
  dwt_write32bitreg(RF_CONF_ID,
                    RF_CONF_TXPLLPOWEN_MASK); // Enable LDO and RF PLL blocks
  dwt_write32bitreg(RF_CONF_ID,
                    RF_CONF_TXALLEN_MASK); // Enable the rest of TX blocks

  //
  // Configure TX clocks
  //
  _dwt_enableclocks(FORCE_SYS_PLL);
  _dwt_enableclocks(FORCE_TX_PLL);

  // Set the frame repetition rate
  if (framerepetitionrate < 4) {
    framerepetitionrate = 4;
  }
  dwt_write32bitoffsetreg(DX_TIME_ID, 0, framerepetitionrate);
  //
  // Configure continuous frame TX
  //
  write_buf[0] = (uint8)(DIAG_TMC_TX_PSTM);
  dwt_writetodevice(DIG_DIAG_ID, DIAG_TMC_OFFSET, 1,
                    write_buf); // Turn the tx power spectrum test mode -
                                // continuous sending of frames
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readtempvbat()
 *
 * @brief this function reads the battery voltage and temperature of the MP
 * The values read here will be the current values sampled by DW1000 AtoD
 *converters.
 * Note on Temperature: the temperature value needs to be converted to give the
 *real temperature
 * the formula is: 1.13 * reading - 113.0
 * Note on Voltage: the voltage value needs to be converted to give the real
 *voltage
 * the formula is: 0.0057 * reading + 2.3
 *
 * NB: To correctly read the temperature this read should be done with xtal
 *clock
 * however that means that the receiver will be switched off, if receiver needs
 *to be on then
 * the timer is used to make sure the value is stable before reading
 *
 * input parameters:
 * @param fastSPI - set to 1 if SPI rate > than 3MHz is used
 *
 * output parameters
 *
 * returns  (temp_raw<<8)|(vbat_raw)
 */
uint16 dwt_readtempvbat(uint8 fastSPI) {
  uint8 wr_buf[2];
  uint8 vbat_raw;
  uint8 temp_raw;

  // These writes should be single writes and in sequence
  wr_buf[0] = 0x80; // Enable TLD Bias
  dwt_writetodevice(RF_CONF_ID, 0x11, 1, wr_buf);

  wr_buf[0] = 0x0A; // Enable TLD Bias and ADC Bias
  dwt_writetodevice(RF_CONF_ID, 0x12, 1, wr_buf);

  wr_buf[0] = 0x0f; // Enable Outputs (only after Biases are up and running)
  dwt_writetodevice(RF_CONF_ID, 0x12, 1, wr_buf); //

  // Reading All SAR inputs
  wr_buf[0] = 0x00;
  dwt_writetodevice(TX_CAL_ID, TC_SARL_SAR_C, 1, wr_buf);
  wr_buf[0] = 0x01; // Set SAR enable
  dwt_writetodevice(TX_CAL_ID, TC_SARL_SAR_C, 1, wr_buf);

  if (fastSPI == 1) {
    deca_sleep(
        1); // If using PLL clocks(and fast SPI rate) then this sleep is needed
    // Read voltage and temperature.
    dwt_readfromdevice(TX_CAL_ID, TC_SARL_SAR_LVBAT_OFFSET, 2, wr_buf);
  } else // change to a slow clock
  {
    _dwt_enableclocks(FORCE_SYS_XTI); // NOTE: set system clock to XTI - this is
                                      // necessary to make sure the values read
                                      // are reliable
    // Read voltage and temperature.
    dwt_readfromdevice(TX_CAL_ID, TC_SARL_SAR_LVBAT_OFFSET, 2, wr_buf);
    // Default clocks (ENABLE_ALL_SEQ)
    _dwt_enableclocks(ENABLE_ALL_SEQ); // Enable clocks for sequencing
  }

  vbat_raw = wr_buf[0];
  temp_raw = wr_buf[1];

  wr_buf[0] = 0x00; // Clear SAR enable
  dwt_writetodevice(TX_CAL_ID, TC_SARL_SAR_C, 1, wr_buf);

  return ((temp_raw << 8) | (vbat_raw));
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readwakeuptemp()
 *
 * @brief this function reads the temperature of the DW1000 that was sampled
 * on waking from Sleep/Deepsleep. They are not current values, but read on last
 * wakeup if DWT_TANDV bit is set in mode parameter of dwt_configuresleep
 *
 * input parameters:
 *
 * output parameters:
 *
 * returns: 8-bit raw temperature sensor value
 */
uint8 dwt_readwakeuptemp(void) {
  uint8 temp_raw;
  dwt_readfromdevice(TX_CAL_ID, TC_SARL_SAR_LTEMP_OFFSET, 1, &temp_raw);
  return (temp_raw);
}

/*!
 *------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readwakeupvbat()
 *
 * @brief this function reads the battery voltage of the DW1000 that was sampled
 * on waking from Sleep/Deepsleep. They are not current values, but read on last
 * wakeup if DWT_TANDV bit is set in mode parameter of dwt_configuresleep
 *
 * input parameters:
 *
 * output parameters:
 *
 * returns: 8-bit raw battery voltage sensor value
 */
uint8 dwt_readwakeupvbat(void) {
  uint8 vbat_raw;
  dwt_readfromdevice(TX_CAL_ID, TC_SARL_SAR_LVBAT_OFFSET, 1, &vbat_raw);
  return (vbat_raw);
}

/* ===============================================================================================
   List of expected (known) device ID handled by this software
   ===============================================================================================

    0xDECA0130                               // DW1000 - MP

   ===============================================================================================
*/
