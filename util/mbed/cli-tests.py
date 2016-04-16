#!/usr/bin/env python2

import os
import sys
import time
import serial
import tempfile
import usb.core
import usb.util

device = '/dev/mbed0'
dev = usb.core.find(idVendor=0x0d28, idProduct=0x0204)


def cli_cmd(ser, cmd):
    # send a command
    buf = ''
    ser.write('{}\r'.format(cmd))
    ser.flush()
    while ser.inWaiting():
        buf += ser.read(s.inWaiting())
        time.sleep(0.2)
    time.sleep(0.4)
    buf += ser.read(s.inWaiting())
    return buf


def cli_call(ser, cmd):
    res = cli_cmd(ser, cmd).splitlines()
    # we cheat and use the last line for showing the PS1 format
    try:
        print(res[-1] + res[0])
        for l in res[1:-1]:
            print(l)
    except IndexError:
        print(' '.join(res[:-1]))


if os.getuid() != 0:
    print('must run with root privileges')
    sys.exit(1)

# was it found?
if dev is None:
    raise ValueError('Device not found')

# set the active configuration. With no arguments, the first
# configuration will be the active one
try:
    dev.set_configuration()
except usb.core.USBError:
    pass

intf = 0
if dev.is_kernel_driver_active(intf):
    dev.detach_kernel_driver(intf)
    usb.util.claim_interface(dev, intf)

try:
    with serial.Serial(device, timeout=1.5, baudrate=57600) as s:
        # cleanup for a fresh connection
        s.send_break()
        s.reset_input_buffer()
        time.sleep(0.1)

        # read the startup logs
        while s.inWaiting():
            s.read(s.inWaiting())
            time.sleep(0.25)
        time.sleep(2)
        s.read(s.inWaiting())

        cli_cmd(s, 'su cli-test')
        cli_cmd(s, 'hostname testbot')
        print('Testing valid CLI calls.')
        cli_call(s, 'loglvl')
        cli_call(s, 'loglvl +')
        cli_call(s, 'loglvl -')
        cli_call(s, 'loglvl ++++')

        cli_call(s, 'help')
        cli_call(s, 'h')
        cli_call(s, '?')
        cli_call(s, 'help help')
        cli_call(s, 'help --list')
        cli_call(s, 'help -l')
        cli_call(s, 'help --all')
        cli_call(s, 'help -a')

        cli_call(s, 'alias')
        cli_call(s, 'a')
        cli_call(s, 'alias help')

        cli_call(s, 'baudrate')
        cli_call(s, 'baud')
        cli_call(s, 'baudrate --list')
        cli_call(s, 'baudrate -l')

        cli_call(s, 'ls')
        cli_call(s, 'l')
        cli_call(s, 'ls /local')

        cli_call(s, 'ps')

        cli_call(s, 'isconn')

        cli_call(s, 'info')
        cli_call(s, 'i')

        cli_call(s, 'rpc /')

        cli_call(s, 'echo test')
        cli_call(s, 'echo TEST')
        cli_call(s, 'echo $test')
        cli_call(s, 'echo $TEST')
        cli_call(s, 'echo \'test\'')
        cli_call(s, 'echo "test"')

        cli_call(s, 'led state on')
        cli_call(s, 'led state off')
        cli_call(s, 'led bright .6')
        cli_call(s, 'led bright 0.1')
        cli_call(s, 'led color white')
        cli_call(s, 'led color blue')
        cli_call(s, 'led color red')
        cli_call(s, 'led color green')
        cli_call(s, 'led color purple')
        cli_call(s, 'led color yellow')

        cli_call(s, 'motors on')
        cli_call(s, 'motors off')
        cli_call(s, 'motors show')
        cli_call(s, 'motors set 0 0')
        cli_call(s, 'motors set 1 50')
        cli_call(s, 'motors set 2 50')
        cli_call(s, 'motors set 3 50')
        cli_call(s, 'motors set 4 50')
        cli_call(s, 'motors show')

        cli_call(s, 'radio show')
        cli_call(s, 'radio set up 3')
        cli_call(s, 'radio set up 5')
        cli_call(s, 'radio test-tx')
        cli_call(s, 'radio test-rx')
        cli_call(s, 'radio test-tx 3')
        cli_call(s, 'radio test-rx 5')
        cli_call(s, 'radio test-tx 8')
        cli_call(s, 'radio test-rx 9')
        cli_call(s, 'radio set down 3')
        cli_call(s, 'radio set down 5')
        cli_call(s, 'radio strobe 2')
        cli_call(s, 'radio strobe 4')
        cli_call(s, 'radio debug')
        cli_call(s, 'radio strobe 2')
        cli_call(s, 'radio strobe 4')
        cli_call(s, 'radio debug')
        cli_call(s, 'radio stress-test 2 30 20')
        cli_call(s, 'radio stress-test 5 30 60')
        time.sleep(0.5)
        cli_call(s, 'radio loopback')
        cli_call(s, 'radio loopback 2')
        time.sleep(1)
        cli_call(s, 'radio')
        cli_call(s, 'radio set reset 3')
        cli_call(s, 'radio')

        print('Testing multiple inline calls.')
        cli_call(s, 'info; help; info')

        print('Testing invalid CLI calls. All following calls should safely fail.')
        cli_call(s, 'su')
        cli_call(s, 'alias notacommand')
        cli_call(s, 'help notacommand')
        cli_call(s, 'ls -wat')
        cli_call(s, 'ls /')
        cli_call(s, 'ps -wat')
        cli_call(s, 'isconn -wat')
        cli_call(s, 'info -wat')
        cli_call(s, 'rpc')
        cli_call(s, 'rpc notanrpccall')
        cli_call(s, 'led')
        cli_call(s, 'led blowup')
        cli_call(s, 'led state notastate')
        cli_call(s, 'led bright 2')
        cli_call(s, 'led bright 1.1')
        cli_call(s, 'led color notacolor')
        cli_call(s, 'radio test-tx 3')
        cli_call(s, 'radio test-rx 5')
        cli_call(s, 'radio set up 25')
        cli_call(s, 'loglvl ++++++++++')
        cli_call(s, 'loglvl ----------')

except serial.serialutil.SerialException as e:
    print(e)

try:
    usb.util.release_interface(dev, intf)
    dev.attach_kernel_driver(intf)
except:
    pass
