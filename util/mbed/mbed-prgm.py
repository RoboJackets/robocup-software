import sys, os, subprocess, signal
from time import sleep
from shutil import copy
from os.path import join, abspath, dirname

# Import pyserial: https://pypi.python.org/pypi/pyserial
from serial import Serial
import serial.tools.list_ports as Ports
from optparse import OptionParser
import pyOCD
from pyOCD.board import MbedBoard

# Setup an interrupt callback so if the script has to be killed, it will 
# still show what we know at that moment in time.
startupLogs = ""
def signal_handler(signal, frame):
    print(startupLogs)
    sys.exit(0)

# Command line argument options
parser = OptionParser()
parser.add_option("-f", "--file", type="string", dest="filename",
                  help="binary file sent to the mbed")
parser.add_option("-d", "--destination", type="string", dest="destination",
                  help="where the binary file should be copied")
parser.add_option("-b", "--baudrate", type="int" ,dest="baud",
                  help="baudrate used for the serial line", default=9600)
parser.add_option("-n", "--no-fat-write", action="store_true", dest="no_mbed_write",
                  help="this flag will prevent from writing the binary to the mbed's USB storage area", default=False)
parser.add_option("-p", "--get-path", action="store_true", dest="return_path",
                  help="returns the path to the first connected mbed's serial port", default=False)
parser.add_option("-l", "--load-memory", action="store_true", dest="load_bin",
                  help="loads the binary directly into the microcontroller's flash memory", default=False)
parser.add_option("-w", "--wait", action="store_true", dest="block_serial",
                  help="waits until a result is sent back from the mbed until closing the serial port", default=False)
(options, args) = parser.parse_args()

# The regex expression for finding mbed devices
found_ports = list(Ports.grep(".*USB VID:PID=0d28:0204.*"))

if options.return_path == True:
    # not sure how things hold up with more than one connected
    # mbed, so returning nothing if there's anything besides one.
    if len(found_ports) == 1:
        sys.exit(found_ports[0][0])
    else:
        sys.exit()

thisDir = os.path.dirname(os.path.abspath(__file__))

if options.filename:
    srcFile = os.path.abspath(options.filename)

if options.destination:
    destPath = os.path.abspath(options.destination)

print(str(len(found_ports)) + " mbed(s) found")

if options.no_mbed_write == False:
    # Move the binary file to the disk storage
    print("loading '" + str(srcFile) + "' to '" + str(destPath) + "'")
    copy(srcFile, destPath)
 
# Reset all of the mbed(s)
if found_ports:
    for i in found_ports:
        serial = Serial(i[0], timeout = 1)
        serial.setBaudrate(options.baud)
        serial.flushInput()
        serial.flushOutput()
        break_time = 0.1

        try:
            serial.sendBreak(break_time)
        except:
            serial.setBreak(False)

        sleep(break_time)
        serial.close()

    print("--  '" + str(os.path.basename(srcFile)) + "' copied to mbed")

    if options.load_bin:
        # We use the pyOCD library here for interfacing with the microcontroller's core directly
        # this gives us 100% certainity of what we control
        try:
            board = MbedBoard.chooseBoard()
        except:
            # sometimes the mbed has a problem with connecting to the board
            # reformatting the FAT drive fixes the issue. This also fixes issues 
            # if there's an active screen session with the mbed
            reformat_script = os.path.join(thisDir, "mbed-reformat.sh")
            # success = subprocess.call(reformat_script, shell=True)
            
            # This script is sketcky
            success = 0
            if success == 1:
                board = MbedBoard.chooseBoard()
            else:
                print("--  unable to find mounted mbed. Mount mbed and try again")
                sys.exit(0)

        target = board.target
        flash = board.flash

        # Half the core of the MCU & load directly into flash
        print("--  loading binary directly into memory...")
        target.halt()
        flash.flashBinary(srcFile)
        target.resetStopOnReset()
        print("    done")
        
        # Now we reset it and setup a serial connection for reading the startup logs
        startup = Serial(str(found_ports[0][0]), timeout = 2)
        startup.setBaudrate(options.baud)
        startup.flushInput()
        startup.flushOutput()

        if options.block_serial:
            startup.timeout = 0;

        # Now we let everything run
        print("--  starting program")
        target.reset()
        sleep(3)

        signal.signal(signal.SIGINT, signal_handler)
        canBlock = False

        # Once we receive the first byte, wait for a timeout period and print out everything that was received
        while(True):
            startupLogs += str(startup.read(256))

            # break once we've stopped receiving serial data
            if startup.inWaiting() == 0 and not (options.block_serial and canBlock):
                break

            if startupLogs[:5] == "START":
                canBlock = True;

            if options.block_serial and startupLogs[-4:] == "DONE":
                # Trim to the expected result response
                startIndex = startupLogs.find("START") + 5
                startupLogs = startupLogs[startIndex:-4] + "\n"
                break

        # Show some additional breaks in the output if we're not running a hardware test
        if not options.block_serial:
            print("========== BEGIN STARTUP LOGS ==========")
        else:
            # otherwise we just put a blank line for the test results
            print("")
        
        print(startupLogs)

        if not options.block_serial:
            print("=========== END STARTUP LOGS ===========")

else:
    print("--  no mbed(s) found")
    sys.exit(0)
