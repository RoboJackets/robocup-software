#!/usr/bin/env python

# import os
# import subprocess
# import xml.etree.ElementTree as et
import psutil

# def runCmd(proc):
#     p = subprocess.Popen(proc.split(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
#     while (True):
#         retcode = p.poll()
#         line = p.stdout.readline()
#         yield line
#         if (retcode is not None):
#             break

# Query the USB devices in XML format
# xmlData = ''
# for l in runCmd('system_profiler -xml SPUSBDataType'):
#     xmlData += str(l.replace('\n', '').replace('\t', '').replace(' ', ''))

# xmlTree = et.fromstring(xmlData)
# for c in xmlTree:
#     print("{} => {}".format(c.tag, c.attrib))

# Query the system block device partitions
diskData = psutil.disk_partitions()
print(diskData)

