#!/usr/bin/env python

from __future__ import print_function

import sys
import os
from os.path import join, abspath, dirname, isdir

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
    from workspace_tools.targets import TARGET_NAMES, TARGET_MAP

    # restore stdout
    sys.stdout = save_stdout

    # print a listing of available targets
    print('set(MBED_AVAILABLE_TARGETS', end='')
    printList(TARGET_NAMES)
    print(')')

    # for each target, set cmake variables for it, using the target
    # as a prefix for what the list holds
    for n in TARGET_NAMES:
        t = TARGET_MAP[n]
        print('set(MBED_TARGET_CORE_' + n, end='')
        if t.core:
            printList(t.core)
        print(')')

        print('set(MBED_TARGET_LABELS_' + n, end='')
        if t.extra_labels:
            printList(t.extra_labels)
        print(')')

        print('set(MBED_TARGET_TOOLCHAINS_' + n, end='')
        if t.supported_toolchains:
            printList(t.supported_toolchains)
        print(')')

        print('set(MBED_TARGET_CODE_' + n, end='')
        if t.detect_code:
            printList(t.detect_code)
        print(')')

        if hasattr(t, 'progen_target'):
            print('set(MBED_TARGET_PROGEN_' + n, end='')
            if t.progen_target:
                printList(t.progen_target)
            print(')')

        if hasattr(t, 'macros'):
            print('set(MBED_TARGET_MACROS_' + n, end='')
            if t.macros:
                printList(t.macros)
            print(')')

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Returns a list of officially supported target for the mbed SDK build system')
    parser.add_argument('-p', '--path', help='The path to the mbed repository root', required=True)
    args = parser.parse_args()

    echoCmakeVars(args.path)
