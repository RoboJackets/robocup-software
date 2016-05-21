import serial
import os


class SAM7:
    def __init__(self, device):
        self.f = serial.Serial(device, timeout=10)

    def __del__(self):
        self.close()

    def close(self):
        if self.f:
            self.f.close()
            self.f = None

    def expect(self, s):
        for want in s:
            got = self.f.read(1)
            if got != want:
                raise AssertionError('Expected 0x%02x, got 0x%02x' %
                                     (ord(want), ord(got)))

    def commandWithLine(self, cmd):
        self.f.write(cmd)
        self.expect('\n\r')
        value = self.f.readline().strip()
        self.expect('\r>')
        return value

    def command(self, cmd):
        self.f.write(cmd)
        self.expect('\n\r>')

    def version(self):
        return self.commandWithLine('V#')

    def readWord(self, addr):
        value = self.commandWithLine('w%x,#' % addr)
        return int(value, 16)

    def readHalf(self, addr):
        value = self.commandWithLine('h%x,#' % addr)
        return int(value, 16)

    def readByte(self, addr):
        value = self.commandWithLine('o%x,#' % addr)
        return int(value, 16)

    def writeWord(self, addr, value):
        self.command('W%x,%x#' % (addr, value))

    def writeHalf(self, addr, value):
        self.command('H%x,%x#' % (addr, value))

    def writeByte(self, addr, value):
        self.command('O%x,%x#' % (addr, value))

    def readData(self, addr, size):
        self.f.write('R%x,%x#' % (addr, size))
        self.expect('\n\r')
        data = ''
        while len(data) < size:
            chunk = self.f.read(size - len(data))
            data += chunk
        self.expect('>')
        return data

    def writeData(self, addr, data):
        self.f.write('S%x,%x#' % (addr, len(data)))
        self.expect('\n\r')
        self.f.write(data)
        assert self.f.read(1) == '>'

    def go(self, addr):
        self.f.write('G%x#' % addr)
        self.expect('\n\r')

    def dump(self, addr, size, filename):
        f_out = file(filename, 'w')
        f_out.write(self.readData(addr, size))
        f_out.close()

    def load(self, addr, filename):
        f_in = file(filename, 'r')
        data = f_in.read()
        f_in.close()
        self.writeData(addr, data)


FLASH = 0x100000
RAM = 0x200000

# MC_FCR values
MC_FMR = 0xffffff60
MC_FCR = 0xffffff64
MC_FSR = 0xffffff68
FCR_KEY = 0x5a000000
# Flash commands
FCR_WP = 0x01
FCR_SLB = 0x02
FCR_WPL = 0x03
FCR_CLB = 0x04
FCR_EA = 0x08
FCR_SGPB = 0x0b
FCR_CGPB = 0x0d
FCR_SSB = 0x0f
