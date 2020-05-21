#!/usr/bin/env python2

from __future__ import print_function

import os
import sys
import time
import shutil
import serial
import argparse
import mbed_lstools as mbedls
from os.path import abspath, isfile
from subprocess import check_call

# parse in the given file(s)
parser = argparse.ArgumentParser(
    description='copies one or more given files to all connected mbed devices.'
)
parser.add_argument('files',
                    nargs='+',
                    help='files to move to all connected mbeds')
parser.add_argument(
    '--mbed_index',
    '-i',
    type=int,
    help='Index of mbed to copy to.  If unset, copies to all mbeds.')
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
        check_call(['sync'], shell=True)


# only select the files that exist from what was given
files = []
for file in args.files:
    f = abspath(file)
    if isfile(f):
        files.append(f)
    else:
        print('{} does not exist'.format(file))

# get the array of mbeds that are connected
mbed_tool = mbedls.create()
mbed_tool.list_unmounted = True
mbeds = mbed_tool.list_mbeds()

if len(mbeds) == 0:
    print("No mbeds found", file=sys.stderr)
    sys.exit(1)


def find_mbed_disk(mbed):
    dirpath = '/dev/disk/by-id'
    for path in os.listdir(dirpath):
        if mbed['target_id'] in path:
            return os.path.join(dirpath, path)
    return None


# sort alphabetically by serial port
mbeds.sort(key=lambda m: m['serial_port'])

# if args.mbed_index is set, copy only to that one
if args.mbed_index != None:
    index = args.mbed_index
    if index < len(mbeds):
        mbeds = [mbeds[index]]
    else:
        print("Unable to find mbed at index '%d'" % index, file=sys.stderr)
        sys.exit(1)

# iterate copying all files to all mbeds
for i in range(len(mbeds)):
    mbed = mbeds[i]
    mount_point = mbed['mount_point']

    print("Using mbed at '%s'" % mbed['serial_port'])

    already_mounted = mount_point != None

    if not already_mounted:
        mount_point = '/mnt/mbed%d' % i
        print("Mounting mbed to '%s'..." % mount_point)
        check_call(['mkdir', '-p', mount_point])
        check_call(['mount', find_mbed_disk(mbed), mount_point])

    for f in files:
        print("Copying '{}' to '{}'...".format(os.path.basename(f),
                                               mount_point))
        shutil.copy2(f, mount_point)
        sync_os()

    # unmount if needed
    if not already_mounted:
        print("Unmounting mbed...")
        check_call(['umount', mount_point])

    # reset the mbed
    print("Rebooting mbed...")
    try:
        ss = serial.Serial(mbed['serial_port'], baudrate=57600)
        ss.sendBreak()
        time.sleep(1)
        ss.sendBreak()
    except serial.serialutil.SerialException as exc:
        print(
            "Unable to open MBED serial port.  Are you already connected to it via the 'screen' program?"
        )
        sys.exit(1)
