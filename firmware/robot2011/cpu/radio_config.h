const uint8_t cc1101_regs[] = {
    0x0b, 0x0c,  // FSCTRL1  - Frequency synthesizer control.
    0x0d, 0x21,  // FREQ2    - Frequency control word, high byte.
    0x0e, 0x7b,  // FREQ1    - Frequency control word, middle byte.
    0x0f, 0x42,  // FREQ0    - Frequency control word, low byte.
    0x10, 0x2d,  // MDMCFG4  - Modem configuration.
    0x11, 0x2f,  // MDMCFG3  - Modem configuration.
    0x12, 0x13,  // MDMCFG2  - Modem configuration.
    0x13, 0x22,  // MDMCFG1  - Modem configuration.
    0x14, 0xe5,  // MDMCFG0  - Modem configuration.
                 //     0x0a, 0x00,    // CHANNR   - Channel number.
    0x15, 0x62,  // DEVIATN  - Modem deviation setting (when FSK modulation is
                 // enabled).
    0x21, 0xb6,  // FREND1   - Front end RX configuration.
    0x22, 0x10,  // FREND0   - Front end RX configuration.
    0x18, 0x18,  // MCSM0    - Main Radio Control State Machine configuration.
    // FIXME - If RXOFF != 0 and we transmit, the radio does not return to RX.
    0x17, 0x03,  // MCSM1    - Main Radio Control State Machine configuration.
    0x19, 0x5d,  // FOCCFG   - Frequency Offset Compensation Configuration.
    0x1a, 0x1c,  // BSCFG    - Bit synchronization Configuration.
    0x1b, 0xc7,  // AGCCTRL2 - AGC control.
    0x1c, 0x00,  // AGCCTRL1 - AGC control.
    0x1d, 0xb0,  // AGCCTRL0 - AGC control.
    0x07, 0x4c,  // PKTCTRL1
    0x08, 0x05,  // PKTCTRL0
    0x06, 0x3d,  // PKTLEN
    0x03, 0x0f,  // FIFOTHR  - RXFIFO and TXFIFO thresholds.
};
