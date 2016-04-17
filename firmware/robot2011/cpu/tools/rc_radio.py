from usb import *

debug = False

device = None
for bus in busses():
    for dev in bus.devices:
        if dev.idVendor == 0x3141 and dev.idProduct == 0x0004:
            device = dev.open()
            break
assert device

device.setConfiguration(1)
device.claimInterface(0)


def autoCalibrate(enabled):
    device.controlMsg(ENDPOINT_IN | TYPE_VENDOR, 4, None, enabled, 0)


cc1101_regs = (
    (0x0b, 0x0c),  # FSCTRL1  - Frequency synthesizer control.
    (0x0d, 0x21),  # FREQ2    - Frequency control word, high byte.
    (0x0e, 0x7b),  # FREQ1    - Frequency control word, middle byte.
    (0x0f, 0x42),  # FREQ0    - Frequency control word, low byte.
    (0x10, 0x2d),  # MDMCFG4  - Modem configuration.
    (0x11, 0x2f),  # MDMCFG3  - Modem configuration.
    (0x12, 0x13),  # MDMCFG2  - Modem configuration.
    (0x13, 0x22),  # MDMCFG1  - Modem configuration.
    (0x14, 0xe5),  # MDMCFG0  - Modem configuration.
    (0x0a, 0x00),  # CHANNR   - Channel number.
    (0x15, 0x62
     ),  # DEVIATN  - Modem deviation setting (when FSK modulation is enabled).
    (0x21, 0xb6),  # FREND1   - Front end RX configuration.
    (0x22, 0x10),  # FREND0   - Front end RX configuration.
    (0x18, 0x08),  # MCSM0    - Main Radio Control State Machine configuration.
    (0x17, 0x00),  # MCSM1    - Main Radio Control State Machine configuration.
    (0x19, 0x5d),  # FOCCFG   - Frequency Offset Compensation Configuration.
    (0x1a, 0x1c),  # BSCFG    - Bit synchronization Configuration.
    (0x1b, 0xc7),  # AGCCTRL2 - AGC control.
    (0x1c, 0x00),  # AGCCTRL1 - AGC control.
    (0x1d, 0xb0),  # AGCCTRL0 - AGC control.
    (0x03, 0x0f),  # FIFOTHR  - RXFIFO and TXFIFO thresholds.
    (0x07, 0x4c),  # PKTCTRL1 - Packet automation control.
    (0x08, 0x05),  # PKTCTRL0 - Packet automation control.
    (0x06, 0x3e)  # PKTLEN   - Packet length.
)


def command(value):
    device.controlMsg(ENDPOINT_IN | TYPE_VENDOR, 2, None, 0, value)


def writeRegister(addr, value):
    device.controlMsg(ENDPOINT_IN | TYPE_VENDOR, 1, None, value, addr)


def configure():
    autoCalibrate(False)
    for (addr, value) in cc1101_regs:
        writeRegister(addr, value)
    autoCalibrate(True)


def channel(n):
    writeRegister(0x0a, n)


def send(buf):
    device.bulkWrite(1, buf)


def receive():
    try:
        buf = device.bulkRead(2, 64, 100)
    except USBError:
        # Timeout
        return None
    return buf[:-2]


configure()
