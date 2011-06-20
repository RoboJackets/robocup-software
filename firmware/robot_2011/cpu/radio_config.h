const uint8_t cc1101_regs[] =
{
    0x0b, 0x0c,    // FSCTRL1  - Frequency synthesizer control.
//    0x0c, 0x00,    // FSCTRL0  - Frequency synthesizer control.
    0x0d, 0x21,    // FREQ2    - Frequency control word, high byte.
    0x0e, 0x7b,    // FREQ1    - Frequency control word, middle byte.
    0x0f, 0x42,    // FREQ0    - Frequency control word, low byte.
    0x10, 0x2d,    // MDMCFG4  - Modem configuration.
    0x11, 0x2f,    // MDMCFG3  - Modem configuration.
    0x12, 0x13,    // MDMCFG2  - Modem configuration.
    0x13, 0x22,    // MDMCFG1  - Modem configuration.
    0x14, 0xe5,    // MDMCFG0  - Modem configuration.
    0x0a, 0x00,    // CHANNR   - Channel number.
    0x15, 0x62,    // DEVIATN  - Modem deviation setting (when FSK modulation is enabled).
    0x21, 0xb6,    // FREND1   - Front end RX configuration.
    0x22, 0x10,    // FREND0   - Front end RX configuration.
    0x18, 0x18,    // MCSM0    - Main Radio Control State Machine configuration.
    0x17, 0x00,    // MCSM1    - Main Radio Control State Machine configuration.
    0x19, 0x1d,    // FOCCFG   - Frequency Offset Compensation Configuration.
    0x1a, 0x1c,    // BSCFG    - Bit synchronization Configuration.
    0x1b, 0xc7,    // AGCCTRL2 - AGC control.
    0x1c, 0x00,    // AGCCTRL1 - AGC control.
    0x1d, 0xb0,    // AGCCTRL0 - AGC control.
    0x07, 0x4c,          // PKTCTRL1
    0x08, 0x05,          // PKTCTRL0
    0x06, 0x3e,   // PKTLEN
//    0x23, 0xea,    // FSCAL3   - Frequency synthesizer calibration.
//    0x24, 0x2a,    // FSCAL2   - Frequency synthesizer calibration.
//    0x25, 0x00,    // FSCAL1   - Frequency synthesizer calibration.
//    0x26, 0x1f,    // FSCAL0   - Frequency synthesizer calibration.
//    0x29, 0x59,    // FSTEST   - Frequency synthesizer calibration.
//    0x2c, 0x88,    // TEST2    - Various test settings.
//    0x2d, 0x31,    // TEST1    - Various test settings.
//    0x2e, 0x09,    // TEST0    - Various test settings.
    0x03, 0x0f,    // FIFOTHR  - RXFIFO and TXFIFO thresholds.
//    0x00, 0x0b,    // IOCFG2   - GDO2 output pin configuration.
//    0x02, 0x0c,    // IOCFG0D  - GDO0 output pin configuration. Refer to SmartRF� Studio User Manual for detailed pseudo register explanation.
//    0x07, 0x04,    // PKTCTRL1 - Packet automation control.
//    0x08, 0x12,    // PKTCTRL0 - Packet automation control.
//    0x09, 0x00,    // ADDR     - Device address.
//    0x06, 0xff,    // PKTLEN   - Packet length.
};
