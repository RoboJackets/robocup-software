#!/usr/bin/env python2

from __future__ import print_function

import sys
import os
import time
import datetime
from os.path import join, abspath, dirname, isdir, basename

'''
A simple python script that uses the official mbed SDK to generate a complete
set of variables for cmake.
'''

def printList(a):
    if isinstance(a, list):
        for i in a:
            print(' "' + str(i), end='"')
    else:
        print(' "' + str(a), end='"')

def echoCmakeVars(mbed_path):
    # Be sure that the workspace_tools directory is in the search path

    # redirect stdout so the warning messages from importing the mbed stuff won't show
    save_stdout = sys.stdout
    sys.stdout = open(os.devnull, 'w')

    # first, set what we think the repo root is
    ROOT = abspath(mbed_path)
    # we check if the path was the mbed repo or the workspace_tools directory
    # if we are in the root of the mbed repo, we import things slightly differently
    if not isdir(join(mbed_path, "workspace_tools")):
        ROOT = abspath(join(dirname(mbed_path), '..'))

    # now add the repo root to the path
    sys.path.insert(0, ROOT)
    from workspace_tools.targets import TARGET_NAMES, TARGET_MAP, CORE_LABELS

    # restore stdout
    sys.stdout = save_stdout

    ts = time.time()
    dt = datetime.datetime.fromtimestamp(ts).strftime("%A %d, %B %Y")
    print('#')
    print('# ***** DO NOT MANUALLY EDIT THIS FILE *****')
    print('#')
    print('# This file was generated using \'{}\' on {}.'.format(basename(abspath(__file__)), dt))
    print('# ')

    # print a listing of available targets
    print('set(MBED_AVAILABLE_TARGETS', end='')
    printList(TARGET_NAMES)
    print(')')

    # for each target, set cmake variables for it, using the target
    # as a prefix for what the list holds
    for n in TARGET_NAMES:
        # set the target map for it's parameters
        t = TARGET_MAP[n]
        # print the target's core info
        if t.core:
            print('set(MBED_TARGET_CORE_' + n, end='')
            printList(t.core)
            print(')')
            # print separate variables for what each index means
            for i, l in enumerate(CORE_LABELS[t.core]):
                # the instruction set of the core CPU
                if i is 0:
                    print('set(MBED_TARGET_ISA_' + n, end='')
                    printList(l)
                    print(')')
                # the architecture of the core CPU
                if i is 1:
                    print('set(MBED_TARGET_ARCH_' + n, end='')
                    printList(l)
                    print(')')
                # not exactly sure where this is used, most cores
                # don't have this set
                if i is 2:
                    print('set(MBED_TARGET_RTOS_ARCHS_' + n, end='')
                    printList(l)
                    print(')')

        # print variables for what the extra labels mean for each target
        if t.extra_labels:
            for i, l in enumerate(t.extra_labels):
                # the microcontroller's vendor
                if i is 0:
                    print('set(MBED_TARGET_VENDOR_' + n, end='')
                    printList(l)
                    print(')')
                # the microcontroller's series/family
                if i is 1:
                    print('set(MBED_TARGET_SERIES_' + n, end='')
                    printList(l)
                    print(')')
                # not exactly sure what indexes beyond this point are
                if i is 2:
                    print('set(MBED_TARGET_LABELS_EXTRA_' + n, end='')
                    printList(l)
                    print(')')

        # print the toolchains that are supported by the target
        if t.supported_toolchains:
            print('set(MBED_TARGET_TOOLCHAINS_' + n, end='')
            printList(t.supported_toolchains)
            print(')')

        # print the target's code - this is most likely only
        # used internally for the mbed SDK build system, but
        # printing it out anyways for ensuring an exhaustive set
        if t.detect_code:
            print('set(MBED_TARGET_CODE_' + n, end='')
            printList(t.detect_code)
            print(')')

        # print the target's progen if available
        if hasattr(t, 'progen_target'):
            if t.progen_target:
                print('set(MBED_TARGET_PROGEN_' + n, end='')
                printList(t.progen_target)
                print(')')

        # print the target's macros that should be defined
        # if available
        if hasattr(t, 'macros'):
            if t.macros:
                print('set(MBED_TARGET_MACROS_' + n, end='')
                printList(t.macros)
                print(')')

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Returns a list of officially supported target for the mbed SDK build system')
    parser.add_argument('-p', '--path', help='The path to the mbed repository root', required=True)
    args = parser.parse_args()

    echoCmakeVars(args.path)
