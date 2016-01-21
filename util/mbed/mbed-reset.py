#!/usr/bin/env python

import sys
import serial

print(sys.argv[1])
ss = serial.Serial(sys.argv[1], baudrate=57600)
ss.sendBreak(duration=0.05)
ss.flush()
