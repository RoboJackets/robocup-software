#include "mbed.h"
#include "deca_device_api.h"
#include "deca_regs.h"

Serial pc(USBTX,USBRX);
BusOut LEDs(LED1, LED2, LED3, LED4);

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t config = {
    7,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

static dwt_txconfig_t txconfig = {
    0x93,            /* PG delay. */
    0xD1D1D1D1,      /* TX power. */
};

//
// /* The frame sent in this example is an 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
//  *     - byte 0: frame type (0xC5 for a blink).
//  *     - byte 1: sequence number, incremented for each new frame.
//  *     - byte 2 -> 9: device ID, see NOTE 1 below.
//  *     - byte 10/11: frame check-sum, automatically set by DW1000.  */
// static uint8 tx_msg[] = {0xC5, 0, 0, 0x43, 0x02, 0, 0};

/* The frame sent in this example is a blink encoded as per the ISO/IEC 24730-62:2013 standard. It is a 14-byte frame composed of the following fields:
 *     - byte 0: frame control (0xC5 to indicate a multipurpose frame using 64-bit addressing).
 *     - byte 1: sequence number, incremented for each new frame.
 *     - byte 2 -> 9: device ID, see NOTE 1 below.
 *     - byte 10: encoding header (0x43 to indicate no extended ID, temperature, or battery status is carried in the message).
 *     - byte 11: EXT header (0x02 to indicate tag is listening for a response immediately after this message).
 *     - byte 12/13: frame check-sum, automatically set by DW1000. */
static uint8 tx_msg_tx[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E', 0x43, 0x02, 0, 0};

/* As "TX then wait for a response" example sends a blink message encoded as per the ISO/IEC 24730-62:2013 standard which includes a bit signalling
 * that a response is listened for, this example will respond with a valid frame (that will be ignored anyway) following the same standard. The
 * response is a 21-byte frame composed of the following fields:
 *     - byte 0/1: frame control (0x8C41 to indicate a data frame using 16-bit source addressing and 64-bit destination addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: application ID (0x609A for data frames in this standard).
 *     - byte 5 -> 12: 64-bit destination address.
 *     - byte 13/14: 16-bit source address, hard coded in this example to keep it simple.
 *     - byte 15: function code (0x10 to indicate this is an activity control message).
 *     - byte 16: activity code (0x00 to indicate activity is finished).
 *     - byte 17/18: new tag blink rate.
 *     - byte 19/20: frame check-sum, automatically set by DW1000.  */
static uint8 tx_msg_rx[] = {0x41, 0x8C, 0, 0x9A, 0x60, 0, 0, 0, 0, 0, 0, 0, 0, 'D', 'W', 0x10, 0x00, 0, 0, 0, 0};

/* Indexes to access to sequence number and destination address of the data frame in the tx_msg array. */
#define DATA_FRAME_SN_IDX 2
#define DATA_FRAME_DEST_IDX 5

/* Index to access to sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 1
/* Index to access to source address of the blink frame in the rx_buffer array. */
#define BLINK_FRAME_SRC_IDX 2

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 100

/* Delay from end of transmission to activation of reception, expressed in UWB microseconds (1 uus is 512/499.2 microseconds). See NOTE 2 below. */
#define TX_TO_RX_DELAY_UUS 60

/* Receive response timeout, expressed in UWB microseconds. See NOTE 3 below. */
#define RX_RESP_TO_UUS 5000

/* Buffer to store received frame. See NOTE 4 below. */
#define FRAME_LEN_MAX 127
static uint8 rx_buffer[FRAME_LEN_MAX];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
static uint16 frame_len = 0;

void tx(void) {
  /* Reset and initialise DW1000. See NOTE 5 below.
   * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
   * performance. */
  if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR) {
      pc.printf("INIT FAILED");
      while (1) { };
  }

  /* Configure DW1000. See NOTE 6 below. */
  dwt_configure(&config);
  dwt_configuretxrf(&txconfig);

  /* Set delay to turn reception on after transmission of the frame. See NOTE 2 below. */
  dwt_setrxaftertxdelay(TX_TO_RX_DELAY_UUS);

  /* Set response frame timeout. */
  dwt_setrxtimeout(RX_RESP_TO_UUS);
  while(1) {
    LEDs[abs(tx_msg_tx[BLINK_FRAME_SN_IDX] % 6 - 3)] = 1;
    // LEDs[tx_msg_tx[BLINK_FRAME_SN_IDX] % 4] = 1;
    dwt_writetxdata(sizeof(tx_msg_tx), tx_msg_tx, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_msg_tx), 0, 0); /* Zero offset in TX buffer, no ranging. */

    /* Start transmission, indicating that a response is expected so that reception is enabled immediately after the frame is sent. */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

      /* We assume that the transmission is achieved normally, now poll for reception of a frame or error/timeout. See NOTE 8 below. */
      while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) { };

      if (status_reg & SYS_STATUS_RXFCG) {
      int i;
        // LEDs[tx_msg_tx[BLINK_FRAME_SN_IDX] % 4] = 1;
        /* Clear local RX buffer to avoid having leftovers from previous receptions. This is not necessary but is included here to aid reading
        * the RX buffer. */
        for (i = 0 ; i < FRAME_LEN_MAX; i++ ) {
           rx_buffer[i] = 0;
        }

        /* A frame has been received, copy it to our local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
        if (frame_len <= FRAME_LEN_MAX)
        {
           dwt_readrxdata(rx_buffer, frame_len, 0);
        }

        /* TESTING BREAKPOINT LOCATION #1 */

        /* At this point, received frame can be examined in global "rx_buffer". An actual application would, for example, start by checking that
        * the format and/or data of the response are the expected ones. A developer might put a breakpoint here to examine this frame. */

        /* Clear good RX frame event in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
        tx_msg_tx[BLINK_FRAME_SN_IDX] = rx_buffer[DATA_FRAME_SN_IDX] + 1;
     }
     else
     {
       /* Clear RX error/timeout events in the DW1000 status register. */
       dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
     }


    //  tx_msg[2] = rx_buffer[1];

     /* Execute a delay between transmissions. */
     wait_ms(TX_DELAY_MS);
     LEDs = 0;
     /* Increment the blink frame sequence number (modulo 256). */
     //tx_msg_tx[BLINK_FRAME_SN_IDX] = tx_msg_rx[DATA_FRAME_SN_IDX] + 1;
     //tx_msg_tx[BLINK_FRAME_SN_IDX] = rx_buffer[DATA_FRAME_SN_IDX];
  }
}

void rx(void) {
  if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR) {
      pc.printf("INIT FAILED");
      while (1) { };
  }

  /* Configure DW1000. See NOTE 3 below. */
  dwt_configure(&config);
  dwt_configuretxrf(&txconfig);
  while(1) {
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) { };

    if (status_reg & SYS_STATUS_RXFCG) {
      LEDs[abs((tx_msg_rx[DATA_FRAME_SN_IDX]+1) % 6 - 3)] = 1;
      /* A frame has been received, read it into the local buffer. */
      frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
      if (frame_len <= FRAME_LEN_MAX) {
          dwt_readrxdata(rx_buffer, frame_len, 0);
      }

      /* TESTING BREAKPOINT LOCATION #1 */

      /* Clear good RX frame event in the DW1000 status register. */
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

      /* Validate the frame is the one expected as sent by "TX then wait for a response" example. */
      if ((frame_len == 14) && (rx_buffer[0] == 0xC5) && (rx_buffer[10] == 0x43) && (rx_buffer[11] == 0x2)) {
        // LEDs[3] = 1;
        tx_msg_rx[DATA_FRAME_SN_IDX] = rx_buffer[BLINK_FRAME_SN_IDX];
        int i;

        /* Copy source address of blink in response destination address. */
        for (i = 0; i < 8; i++) {
          tx_msg_rx[DATA_FRAME_DEST_IDX + i] = rx_buffer[BLINK_FRAME_SRC_IDX + i];
        }

        /* Write response frame data to DW1000 and prepare transmission. See NOTE 6 below.*/
        dwt_writetxdata(sizeof(tx_msg_rx), tx_msg_rx, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(tx_msg_rx), 0, 0); /* Zero offset in TX buffer, no ranging. */

        /* Send the response. */
        dwt_starttx(DWT_START_TX_IMMEDIATE);

        /* Poll DW1000 until TX frame sent event set. */
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) { };

        /* Clear TX frame sent event. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

        /* Increment the data frame sequence number (modulo 256). */

        // tx_msg_rx[DATA_FRAME_SN_IDX]++;

      }
    }
    else {
      /* Clear RX error events in the DW1000 status register. */
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
    }
    LEDs = 0;
  }
}

int main(void) {
  rx();

  // if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR) {
  //   lcd_display_str("INIT FAILED");
  //   while (1) { };
  // }
  // dwt_configure(&config);
  //
  // /* Set delay to turn reception on after transmission of the frame. See NOTE 2 below. */
  // dwt_setrxaftertxdelay(TX_TO_RX_DELAY_UUS);
  //
  // /* Set response frame timeout. */
  // dwt_setrxtimeout(RX_RESP_TO_UUS);
  //
  // while(1) {
  //   LEDs[tx_msg[2]] = 1;
  //   tx();
  //   //LEDs[3] = 1;
  //   rx();
  //   LEDs[tx_msg[2]] = 0;
  //   tx_msg[2] = rx_buffer[2]%4;
  // }

  /*
  while(1) {
    rx();
    LEDs[tx_msg[2]] = 0;
    tx_msg[2] = (rx_buffer[2]+1)%3;
    LEDs[tx_msg[2]] = 1;
    tx();
  }
  */
}