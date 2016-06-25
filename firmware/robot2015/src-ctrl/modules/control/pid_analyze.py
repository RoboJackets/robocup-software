#!/usr/bin/env python3

"""
This script can plot data from the robot's pid controller in order to visualize
the relationship between command velocity, actual velocity, and pid control
values.

In order to gather this data, you must enable the printouts in the
PidMotionController's run() method.  This prints a python dict of values every
time the run() method is called.  You can capture these from the robot's log and
paste them into a new python file in an array called "data". Then use this
script to plot the data.
"""

import matplotlib.pyplot as plt
import sys

if len(sys.argv) != 2:
    print('Usage: graph_data <module>\n\tnote: dont include the ".py"')
    exit(1)

mod = __import__(sys.argv[1])
data = mod.data

tt = range(len(data))

for wheel_idx in range(4):
    w0 = list(map(lambda e: e['wheelVels'][wheel_idx], data))
    tw0 = list(map(lambda e: e['targetWheelVels'][wheel_idx], data))
    d0 = list(map(lambda e: e['duty'][wheel_idx], data))
    plt.subplot(410 + wheel_idx + 1)
    plt.title('Wheel %d' % (wheel_idx + 1))
    # plt.plot(tt, w0, 'r--', tt, tw0, 'go', tt, d0, 'bs')
    plt.plot(tt, w0, 'r--', label='Vel (rad/s)')
    plt.plot(tt, tw0, 'go', label='Target (rad/s)')
    plt.plot(tt, d0, 'bs', label='Duty Cycle ([-511,511])')
    plt.plot(tt, [6 * d for d in tw0], label='Static ctrl duty cycle')

plt.legend()
plt.show()
