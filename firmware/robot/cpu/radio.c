#include "radio.h"
#include "timer.h"
#include "status.h"
#include "radio_config.h"

uint8_t radio_rx_len;
uint8_t radio_rx_buf[64];
int_fast8_t last_rssi;
volatile int radio_in_tx = 0;

static int current_channel;

uint8_t radio_command(uint8_t cmd) {
    radio_select();
    uint8_t status = spi_xfer(cmd);
    radio_deselect();

    return status;
}

uint8_t radio_read(uint8_t addr) {
    radio_select();
    spi_xfer(addr | CC_READ);
    uint8_t value = spi_xfer(SNOP);
    radio_deselect();

    return value;
}

uint8_t radio_write(uint8_t addr, uint8_t value) {
    radio_select();
    uint8_t status = spi_xfer(addr);
    spi_xfer(value);
    radio_deselect();

    return status;
}

void radio_channel(int n) {
    current_channel = n;
    radio_write(CHANNR, n);
}

// Waits for the radio's version byte to have the expected value.
// This verifies that the radio crystal oscillator and SPI interface are
// working.
static int radio_check_version() {
    // It may take some time for the radio's oscillator to start up
    for (int i = 0; i < 10; ++i) {
        if (radio_read(VERSION) == 0x04) {
            return 1;
        }

        delay_ms(10);
    }
    return 0;
}

int radio_init() {
    // Reset
    delay_ms(50);
    radio_command(SRES);
    delay_ms(50);

    if (!radio_check_version()) {
        failures |= Fail_Radio_Interface;
        return 0;
    }

    // Test GDO2 high
    radio_write(IOCFG2, 0x2f | GDOx_INVERT);
    if (!radio_gdo2()) {
        // Interrupt line is stuck low
        failures |= Fail_Radio_Int_Low;
        return 0;
    }

    // Test GDO2 low
    radio_write(IOCFG2, 0x2f);
    if (radio_gdo2()) {
        // Interrupt line is stuck high
        failures |= Fail_Radio_Int_High;
        return 0;
    }

    return 1;
}

void radio_configure() {
    radio_in_tx = 0;
    radio_command(SIDLE);

    radio_select();
    for (int i = 0; i < sizeof(cc1101_regs); ++i) {
        spi_xfer(cc1101_regs[i]);
    }
    radio_deselect();

    radio_write(IOCFG2, 6 | GDOx_INVERT);

    radio_channel(current_channel);

    radio_command(SFRX);
    radio_command(SRX);
}

void radio_transmit(uint8_t* buf, int len) {
    LED_ON(LED_RY);
    radio_select();
    spi_xfer(TXFIFO | CC_BURST);
    spi_xfer(len);
    for (int i = 0; i < len; ++i) {
        spi_xfer(buf[i]);
    }
    radio_deselect();

    // Start transmitting.  When this finishes, the radio will automatically
    // switch to RX
    // without calibrating (because it doesn't go through IDLE).
    radio_command(STX);

    radio_in_tx = 1;
}

static void tx_finished() {
    LED_OFF(LED_RY);
    // 	radio_command(SRX);

    radio_in_tx = 0;
}

static int rx_finished() {
    radio_write(FSCTRL0, radio_read(FREQEST));

    uint8_t bytes = radio_read(RXBYTES);

    if (bytes < 3) {
        // Bad CRC, so the packet was flushed (or the radio got misconfigured).
        radio_command(SFRX);
        radio_command(SRX);
        return 0;
    }

    // Read the packet from the radio
    radio_select();
    spi_xfer(RXFIFO | CC_READ | CC_BURST);
    radio_rx_len = spi_xfer(SNOP);
    if (radio_rx_len > sizeof(radio_rx_buf)) {
        // Either PKTLEN in the radio configuration is wrong or we lost data in
        // the FIFO and this wasn't really a length byte.
        radio_deselect();
        radio_command(SFRX);
        radio_command(SRX);
        radio_rx_len = 0;
        return 0;
    }

    for (int i = 0; i < radio_rx_len; ++i) {
        radio_rx_buf[i] = spi_xfer(SNOP);
    }

    // Read status bytes
    last_rssi = (int8_t)spi_xfer(SNOP);
    uint8_t status = spi_xfer(SNOP);
    radio_deselect();

    if (!(status & 0x80)) {
        // Bad CRC
        //
        // Autoflush is supposed to be on so this should never happen.
        // If we get here and autoflush is on, this means some bytes have been
        // lost
        // and the status byte isn't really the status byte.
        radio_command(SFRX);
        radio_command(SRX);
        return 0;
    }

    return 1;
}

int radio_poll() {
    if ((AT91C_BASE_PIOA->PIO_ISR & RADIO_INT) &&
        (AT91C_BASE_PIOA->PIO_PDSR & RADIO_INT)) {
        if (radio_in_tx) {
            tx_finished();
        } else {
            return rx_finished();
        }
    }

    return 0;
}
