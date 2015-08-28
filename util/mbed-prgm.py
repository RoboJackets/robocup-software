import sys, os
from time import sleep
from shutil import copy
from os.path import join, abspath, dirname

# Import pyserial: https://pypi.python.org/pypi/pyserial
from serial import Serial
import serial.tools.list_ports as Ports
from optparse import OptionParser
import pyOCD
from pyOCD.board import MbedBoard

# Command line argument options
parser = OptionParser()
parser.add_option("-f", "--file", type="string", dest="filename",
                  help="binary file sent to the mbed")
parser.add_option("-d", "--destination", type="string", dest="destination",
                  help="where the binary file should be copied")
parser.add_option("-b", "--baudrate", type="int" ,dest="baud",
                  help="baudrate used for the serial line", default=9600)
(options, args) = parser.parse_args()

# The regex expression for finding mbed devices
found_ports = list(Ports.grep(".*USB VID:PID=0d28:0204.*"))
print str(len(found_ports)) + " mbed(s) found"

srcFile = os.path.abspath(options.filename)
destPath = os.path.abspath(options.destination)

# Move the binary file to the disk storage
print "loading '" + str(srcFile) + "' to '" + str(destPath) + "'"
copy(srcFile, destPath)
# sleep(7)
 
# Reset all of the mbed(s)
if found_ports:
    for i in found_ports:
        try:
            serial = Serial(i[0], timeout = 1)
            serial.setBaudrate(options.baud)
            serial.flushInput()
            serial.flushOutput()

            try:
                #print "--  rebooting mbed"
                serial.sendBreak()
            except:
                serial.setBreak(False)

            sleep(0.3)

            serial.close()

            '''
            numRX = serial.inWaiting()
            if numRX > 0:
                c = serial.read(numRX)
                print str(c)
            '''
        except:
            print "Unexpected error:", sys.exc_info()[0]

    '''
    if len(found_ports) > 1:
        print "--  all mbed writes complete\n"
    else:
        print "--  mbed write complete\n"
    '''

    # We use the pyOCD library here for interfacing with the microcontroller's core directly
    # this gives us 100% certainity of what we control
    board = MbedBoard.chooseBoard()
    target = board.target
    flash = board.flash

    # Half the core of the MCU & load directly into flash
    print "--  loading binary directly into memory"
    flash.flashBinary(srcFile)
    target.resetStopOnReset()
    #target.reset()
    
    # Now we reset it and setup a serial connection for reading the startup logs
    startup = Serial(str(found_ports[0][0]), timeout = 2)
    startup.setBaudrate(options.baud)
    startup.flushInput()
    startup.flushOutput()

    # Now we let everything run
    target.resume()
    sleep(3)

    # Once we receive the first byte, wait for a timeout period and print out everything that was received
    startupLogs = ""
    while(True):
        startupLogs += str(startup.read(256))

        if startup.inWaiting() == 0:
            break

    print startupLogs

    '''
        if numRX > 0:
            print "--  waiting for startup logs to finish"
            # sleep(10)
            c = startup.read(100)
            print c
            break
        else:
            sleep(0.2)
        

    if numRX == 0:
        print "--  no received startup logs"
    '''

else:
    print "-- no mbeds found"
    sys.exit(0)
