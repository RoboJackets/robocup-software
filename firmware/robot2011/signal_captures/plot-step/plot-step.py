#!/usr/bin/env python

import sys
from numpy import *
from matplotlib.pyplot import *

if len(sys.argv) != 2:
    print >> sys.stderr, 'Usage: %s <filename>' % sys.argv[0]
    sys.exit(1)

data = loadtxt(sys.argv[1])
speed = data[1:, 1]
plot(speed)
show()
