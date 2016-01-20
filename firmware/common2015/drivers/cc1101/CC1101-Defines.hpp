#pragma once

/**
* @file CC1101-Defines.h
* @brief This header file defines the registers, strobe, and states of the
* CC1101 Sub-1GHz Transceiver
*/

// REGISTERS
#define CCXXX1_IOCFG2 0x00    // GDO2 output pin configuration
#define CCXXX1_IOCFG1 0x01    // GDO1 output pin configuration
#define CCXXX1_FIFOTHR 0x03   // RX FIFO and TX FIFO thresholds
#define CCXXX1_SYNC1 0x04     // Sync word, high byte
#define CCXXX1_SYNC0 0x05     // Sync word, low byte
#define CCXXX1_IOCFG0 0x02    // GDO0 output pin configuration
#define CCXXX1_FIFOTHR 0x03   // RX FIFO and TX FIFO thresholds
#define CCXXX1_SYNC1 0x04     // Sync word, high byte
#define CCXXX1_SYNC0 0x05     // Sync word, low byte
#define CCXXX1_PCKLEN 0x06    // Packet length
#define CCXXX1_PCKCTRL1 0x07  // Packet automation control
#define CCXXX1_PCKCTRL0 0x08  // Packet automation control
#define CCXXX1_ADDR 0x09      // Device address
#define CCXXX1_CHANNR 0x0A    // Channel number
#define CCXXX1_FSCTRL1 0x0B   // Frequency synthesizer control
#define CCXXX1_FSCTRL0 0x0C   // Frequency synthesizer control
#define CCXXX1_FREQ2 0x0D     // Frequency control word, high byte
#define CCXXX1_FREQ1 0x0E     // Frequency control word, middle byte
#define CCXXX1_FREQ0 0x0F     // Frequency control word, low byte
#define CCXXX1_MDMCFG4 0x10   // Modem configuration
#define CCXXX1_MDMCFG3 0x11   // Modem configuration
#define CCXXX1_MDMCFG2 0x12   // Modem configuration
#define CCXXX1_MDMCFG1 0x13   // Modem configuration
#define CCXXX1_MDMCFG0 0x14   // Modem configuration
#define CCXXX1_DEVIATN 0x15   // Modem deviation setting
#define CCXXX1_MCSM2 0x16     // Main Radio Control State Machine configuration
#define CCXXX1_MCSM1 0x17     // Main Radio Control State Machine configuration
#define CCXXX1_MCSM0 0x18     // Main Radio Control State Machine configuration
#define CCXXX1_FOCCFG 0x19    // Frequency Offset Compensation configuration
#define CCXXX1_BSCFG 0x1A     // Bit Synchronization configuration
#define CCXXX1_AGCCTRL2 0x1B  // AGC control
#define CCXXX1_AGCCTRL1 0x1C  // AGC control
#define CCXXX1_AGCCTRL0 0x1D  // AGC control
#define CCXXX1_WOREVT1 0x1E   // High byte Event 0 timeout
#define CCXXX1_WOREVT0 0x1F   // Low byte Event 0 timeout
#define CCXXX1_WORCTRL 0x20   // Wake On Radio control
#define CCXXX1_FREND1 0x21    // Front end RX configuration
#define CCXXX1_FREND0 0x22    // Front end TX configuration
#define CCXXX1_FSCAL3 0x23    // Frequency synthesizer calibration
#define CCXXX1_FSCAL2 0x24    // Frequency synthesizer calibration
#define CCXXX1_FSCAL1 0x25    // Frequency synthesizer calibration
#define CCXXX1_FSCAL0 0x26    // Frequency synthesizer calibration
#define CCXXX1_RCCTRL1 0x27   // RC oscillator configuration
#define CCXXX1_RCCTRL0 0x28   // RC oscillator configuration
#define CCXXX1_FSTEST 0x29    // Frequency synthesizer calibration control
#define CCXXX1_PTEST 0x2A     // Production test
#define CCXXX1_AGCTEST 0x2B   // AGC test
#define CCXXX1_TEST2 0x2C     // Various test settings
#define CCXXX1_TEST1 0x2D     // Various test settings
#define CCXXX1_TEST0 0x2E     // Various test settings

// STROBE COMMANDS
//(all good for 1201)
#define CCXXX1_SRES 0x30  // Reset chip.
#define CCXXX1_SFSTXON \
    0x31  // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
#define CCXXX1_SXOFF 0x32  // Turn off crystal oscillator.
#define CCXXX1_SCAL 0x33   // Calibrate frequency synthesizer and turn it off
#define CCXXX1_SRX \
    0x34  // Enable RX. Perform calibration first if coming from IDLE and
#define CCXXX1_STX \
    0x35  // In IDLE state: Enable TX. Perform calibration first if
#define CCXXX1_SIDLE \
    0x36  // Exit RX / TX, turn off frequency synthesizer and exit
#define CCXXX1_SAFC 0x37  // Perform AFC adjustment of the frequency synthesizer
#define CCXXX1_SWOR 0x38  // Start automatic RX polling sequence (Wake-on-Radio)
#define CCXXX1_SPWD 0x39  // Enter power down mode when CSn goes high.
#define CCXXX1_SFRX 0x3A  // Flush the RX FIFO buffer.
#define CCXXX1_SFTX 0x3B  // Flush the TX FIFO buffer.
#define CCXXX1_SWORRST 0x3C  // Reset real time clock.

// READ ONLY REGISTERS
#define CCXXX1_SNOP \
    0x3D  // No operation. May be used to pad strobe commands to two bytes for
          // simpler software.
#define CCXXX1_PARTNUM 0x30
#define CCXXX1_VERSION 0x31
#define CCXXX1_FREQEST 0x32
#define CCXXX1_LQI 0x33
#define CCXXX1_RSSI 0x34
#define CCXXX1_MARCSTATE 0x35
#define CCXXX1_WORTIME1 0x36
#define CCXXX1_WORTIME0 0x37
#define CCXXX1_PKTSTATUS 0x38
#define CCXXX1_VCO_VC_DAC 0x39
#define CCXXX1_TXBYTES 0x3A
#define CCXXX1_RXBYTES 0x3B
#define CCXXX1_RCCTRL1_STATUS 0x3C
#define CCXXX1_RCCTRL0_STATUS 0x3D

// POWER REGISTERS
#define CCXXX1_PATABLE 0x3E
#define CCXXX1_TXFIFO 0x3F
#define CCXXX1_RXFIFO 0x3F

// BURST/SINGLE MODIFIERS
#define CCXXX1_WRITE_BURST 0x40
#define CCXXX1_READ_SINGLE 0x80
#define CCXXX1_READ_BURST 0xC0

// GENERAL DEFINES
#define CCXXX1_RXFIFO_MASK 0x7F

// CHIP STATUS
#define CHIP_RDY 0x80
#define CHIP_STATE_MASK 0x70
#define CHIP_STATE_IDLE 0x00
#define CHIP_STATE_RX 0x10
#define CHIP_STATE_TX 0x20
#define CHIP_STATE_FSTON 0x30
#define CHIP_STATE_CALIBRATE 0x40
#define CHIP_STATE_SETTLING 0x50
#define CHIP_STATE_RXFIFO_OVERFLOW 0x60
#define CHIP_STATE_TXFIFO_UNDERFLOW 0x70
#define FIFO_BYTES_MASK 0x0F

// FREQUENCY DEFINITIONS
// #define _902MHZ_ 901833462
// #define _316KHZ_ 316406
#define CCXXX1_IF_FREQUENCY 316406  // 316 kHz
#define CCXXX1_BASE_FREQUENCY 901833462
#define CCXXX1_EXPECTED_VERSION_NUMBER 0x04
#define CCXXX1_CRYSTAL_FREQUENCY 27000000  // 27 MHz

/// RF_SETTINGS is a data structure which contains all relevant CCxxx0 registers
typedef struct rf_settings_t {
    uint8_t FSCTRL1;  // Frequency synthesizer control.
    uint8_t IOCFG0;   // GDO0 output pin configuration
    uint8_t FSCTRL0;  // Frequency synthesizer control.
    uint8_t FREQ2;    // Frequency control word, high byte.
    uint8_t FREQ1;    // Frequency control word, middle byte.
    uint8_t FREQ0;    // Frequency control word, low byte.
    uint8_t MDMCFG4;  // Modem configuration.
    uint8_t MDMCFG3;  // Modem configuration.
    uint8_t MDMCFG2;  // Modem configuration.
    uint8_t MDMCFG1;  // Modem configuration.
    uint8_t MDMCFG0;  // Modem configuration.
    uint8_t CHANNR;   // Channel number.
    uint8_t
        DEVIATN;  // Modem deviation setting (when FSK modulation is enabled).
    uint8_t FREND1;    // Front end RX configuration.
    uint8_t FREND0;    // Front end RX configuration.
    uint8_t MCSM0;     // Main Radio Control State Machine configuration.
    uint8_t MCSM1;     // Main Radio Control State Machine configuration.
    uint8_t MCSM2;     // Main Radio Control State Machine configuration.
    uint8_t FOCCFG;    // Frequency Offset Compensation Configuration.
    uint8_t BSCFG;     // Bit synchronization Configuration.
    uint8_t AGCCTRL2;  // AGC control.
    uint8_t AGCCTRL1;  // AGC control.
    uint8_t AGCCTRL0;  // AGC control.
    uint8_t FSCAL3;    // Frequency synthesizer calibration.
    uint8_t FSCAL2;    // Frequency synthesizer calibration.
    uint8_t FSCAL1;    // Frequency synthesizer calibration.
    uint8_t FSCAL0;    // Frequency synthesizer calibration.
    uint8_t FSTEST;    // Frequency synthesizer calibration control
    uint8_t TEST2;     // Various test settings.
    uint8_t TEST1;     // Various test settings.
    uint8_t TEST0;     // Various test settings.
    uint8_t FIFOTHR;   // RXFIFO and TXFIFO thresholds.
    uint8_t IOCFG2;    // GDO2 output pin configuration
    uint8_t IOCFG1;    // GDO1 output pin configuration
    uint8_t PCKCTRL1;  // Packet automation control.
    uint8_t PCKCTRL0;  // Packet automation control.
    uint8_t ADDR;      // Device address.
    uint8_t PCKLEN;    // Packet length.
} rf_settings_t;

/** Enumerations for state types of the CC1101 */
enum radio_state_t {
    RADIO_IDLE = 0,
    RADIO_RX = 1,
    RADIO_TX = 2,
    RADIO_FSTXON = 3,
    RADIO_CALIBRATE = 4,
    RADIO_SETTLING = 5,
    RADIO_RXFIFO_OVERFLOW = 6,
    RADIO_TXFIFO_OVERFLOW = 7
};

/** Enumerations for packet format settings of the CC1101 */
enum pck_format_t {
    FORMAT_DEFAULT = 0,
    FORMAT_SYNC_SERIAL = 1,
    FORMAT_RAND_TX = 2,
    FORMAT_ASYC_SERIAL = 3
};

/** Enumerations for packet length types of the CC1101 */
enum pck_length_type_t {
    PACKET_FIXED = 0,
    PACKET_VARIABLE = 1,
    PACKET_INFINITE = 2
};

/** Enumerations for packet address checking types of the CC1101 */
enum pck_addr_chk_t {
    ADDR_OFF = 0,
    ADDR_CHK = 1,
    ADDR_CHK_AND_BCAST = 2,
    ADDR_CHK_AND_BCAST_ALL = 3
};

/** Data structure for managing how the CC1101 handels packets */
typedef struct pck_ctrl_t {
    bool whitening_en;
    bool crc_en;
    bool autoflush_en;
    bool status_field_en;
    uint8_t preamble_thresh;
    uint8_t size;
    pck_format_t format_type;
    pck_length_type_t length_type;
    pck_addr_chk_t addr_check;
} pck_ctrl_t;

/** Enumerations for modulation types of the CC1101 */
enum mod_format_t {
    MOD_TWO_FSK = 0,
    MOD_GFSK = 1,
    MOD_ASK = 3,
    MOD_FOUR_FSK = 4,
    MOD_MSK = 7
};

/** Enumerations for signal syncronization of the CC1101 */
enum sync_mode_t {
    SYNC_NONE = 0,
    SYNC_LOW_ALLOW_ONE = 1,
    SYNC_LOW_ALLOW_NONE = 2,
    SYNC_HIGH_ALLOW_TWO = 3,
    SYNC_JUST_CARRIER_SENSE = 4,
    SYNC_LOW_ALLOW_ONE_CS = 5,
    SYNC_LOW_ALLOW_NONE_CS = 6,
    SYNC_HIGH_ALLOW_TWO_CS = 7
};

/** Enumerations for preamble byte sizes of the CC1101 */
enum pream_bytes_t {
    PREAM_TWO = 0,
    PREAM_THREE = 1,
    PREAM_FOUR = 2,
    PREAM_SIX = 3,
    PREAM_EIGHT = 4,
    PREAM_TWELVE = 5,
    PREAM_SIXTEEN = 6,
    PREAM_TWENTY_FOUR = 7
};

/** Data structure for managing how the CC1101 modulates/demodulates signals */
typedef struct modem_t {
    bool dc_filter_off_en;
    bool manchester_encode_en;
    bool fec_en;
    uint8_t data_rate_exp;
    uint8_t data_rate_mtsa;
    uint8_t channel_bw;
    uint8_t channel_bw_exp;
    uint8_t channel_space_exp;
    uint8_t channel_space_mtsa;
    mod_format_t mod_type;
    sync_mode_t sync_mode;
    pream_bytes_t preamble_bytes;
} modem_t;
