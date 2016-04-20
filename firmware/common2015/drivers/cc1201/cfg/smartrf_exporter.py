#!/usr/bin/env python3
"""
This python script reads a TI SmartRF xml file and outputs the registers as a C
header file mapping registers to values.  This functionality is built into
SmartRF, but it's a pain to have to re-export every time a change is made.
"""

import sys
import xml.etree.ElementTree as etree

if len(sys.argv) != 3:
    print("Usage: %s path/to/smartrf.xml outputfile.h" % sys.argv[0],
          file=sys.stderr)
    exit(1)

infile = sys.argv[1]
outfile = sys.argv[2]

root = etree.parse(infile).getroot()
register_settings = root.find('registersettings')

with open(outfile, 'w') as out:
    out.write("// DO NOT MODIFY THIS FILE\n")
    out.write(
        "// It was automatically generated from a smartrf xml file by the smartrf exporter script\n")
    out.write('static const registerSetting_t preferredSettings[] = {\n')
    for reg in register_settings:
        name = reg.find('Name').text
        if name.startswith('AES'): continue  # skip AES registers
        val = reg.find('Value').text

        out.write('    {CC1201_%s, %s},\n' % (name, val))

    out.write('};\n')

print("Wrote output register settings C file to '%s'" % outfile)
