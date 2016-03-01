#!/usr/bin/env python2

import os
import time
import shutil
import serial
import argparse
import mbed_lstools as mbedls
from os.path import abspath, isfile

# parse in the given file(s)
parser = argparse.ArgumentParser(
    description=
    'copies one or more given files to all connected mbed devices.')
parser.add_argument('files',
                    nargs='+',
                    help='files to move to all connected mbeds')
args = parser.parse_args()


def sync_os():
    try:
        if hasattr(os, 'sync'):
            os.sync()
        else:
            import ctypes
            libc = ctypes.CDLL("libc.so.6")
            libc.sync()
    except:
        import subprocess
        subprocess.check_call(['sync'], shell=True)

# only select the files that exist from what was given
files = []
for file in args.files:
    f = abspath(file)
    if isfile(f):
        files.append(f)
    else:
        print('{} does not exist'.format(file))

# get the array of mbeds that are connected
mbeds = mbedls.create().list_mbeds()

# iterate copying all files to all mbeds
for mbed in mbeds:
    # move the files over
    dst = mbed['mount_point']
    for f in files:
        shutil.copy2(f, dst)
        print('copying {} to {}'.format(f, dst))
        sync_os()
    # reset the mbed
    for x in range(2):
        try:
            ss = serial.Serial(mbed['serial_port'], baudrate=57600)
            ss.sendBreak()
            ss.flush()
        except:
            pass
        time.sleep(2)
