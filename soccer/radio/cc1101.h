#pragma once

/* Addresses */
enum {
    /* Flags */
    CC_BURST = 0x40,
    CC_READ = 0x80,
    CC_READONLY = 0xc0, /* Read for read-only registers */

    /* Read/write registers */
    IOCFG2 = 0x00,
    IOCFG1 = 0x01,
    IOCFG0 = 0x02,
    FIFOTHR = 0x03,
    SYNC1 = 0x04,
    SYNC0 = 0x05,
    PKTLEN = 0x06,
    PKTCTRL1 = 0x07,
    PKTCTRL0 = 0x08,
    ADDR = 0x09,
    CHANNR = 0x0a,
    FSCTRL1 = 0x0b,
    FSCTRL0 = 0x0c,
    FREQ2 = 0x0d,
    FREQ1 = 0x0e,
    FREQ0 = 0x0f,
    MDMCFG4 = 0x10,
    MDMCFG3 = 0x11,
    MDMCFG2 = 0x12,
    MDMCFG1 = 0x13,
    MDMCFG0 = 0x14,
    DEVIATN = 0x15,
    MCSM2 = 0x16,
    MCSM1 = 0x17,
    MCSM0 = 0x18,
    FOCCFG = 0x19,
    BSCFG = 0x1a,
    AGCCTRL2 = 0x1b,
    AGCCTRL1 = 0x1c,
    AGCCTRL0 = 0x1d,
    WOREVT1 = 0x1e,
    WOREVT0 = 0x1f,
    WORCTRL = 0x20,
    FREND1 = 0x21,
    FREND0 = 0x22,
    FSCAL3 = 0x23,
    FSCAL2 = 0x24,
    FSCAL1 = 0x25,
    FSCAL0 = 0x26,
    RCCTRL1 = 0x27,
    RCCTRL0 = 0x28,
    FSTEST = 0x29,
    PTEST = 0x2a,
    AGCTEST = 0x2b,
    TEST2 = 0x2c,
    TEST1 = 0x2d,
    TEST0 = 0x2e,
    /* 0x2f is unused */

    /* Command strobes */
    SRES = 0x30,
    SFSTXON = 0x31,
    SXOFF = 0x32,
    SCAL = 0x33,
    SRX = 0x34,
    STX = 0x35,
    SIDLE = 0x36,
    /* 0x37 is unused */
    SWOR = 0x38,
    SPWD = 0x39,
    SFRX = 0x3a,
    SFTX = 0x3b,
    SWORRST = 0x3c,
    SNOP = 0x3d,

    /* Read-only registers */
    PARTNUM = (0x30 | CC_BURST),
    VERSION = (0x31 | CC_BURST),
    FREQEST = (0x32 | CC_BURST),
    LQI = (0x33 | CC_BURST),
    RSSI = (0x34 | CC_BURST),
    MARCSTATE = (0x35 | CC_BURST),
    WORTIME1 = (0x36 | CC_BURST),
    WORTIME0 = (0x37 | CC_BURST),
    PKTSTATUS = (0x38 | CC_BURST),
    VCO_VC_DAC = (0x39 | CC_BURST),
    TXBYTES = (0x3a | CC_BURST),
    RXBYTES = (0x3b | CC_BURST),
    RCCTRL1_STATUS = (0x3c | CC_BURST),
    RCCTRL0_STATUS = (0x3d | CC_BURST),

    /* Multi-byte registers */
    PATABLE = 0x3f,
    TXFIFO = 0x3f, /* Write only */
    RXFIFO = 0x3f  /* Read only */
};

/* IOCFG[2-0] */
enum { GDOx_INVERT = 0x40 };

/* MCSM1 */
enum {
    RXOFF_IDLE = 0x00,
    RXOFF_FSTXON = 0x04,
    RXOFF_TX = 0x08,
    RXOFF_RX = 0x0c,

    TXOFF_IDLE = 0x00,
    TXOFF_FSTXON = 0x01,
    TXOFF_TX = 0x02,
    TXOFF_RX = 0x03
};
