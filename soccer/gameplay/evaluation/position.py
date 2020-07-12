import main
import robocup
import constants
import math
'''returns all robots, as well as their average positions, that are within a given radius of the ball'''


def ball_vicinity(radius):
    contending = []

    for u in main.our_robots:
        dist = (main.ball().pos - u.pos).mag()
        if dist <= radius:
            contending.append(u)

    for t in main.their_robots:
        dist = (main.ball().pos - t.pos).mag()
        if dist <= radius:
            contending.append(t)

    cpos = 0  #position sum to calculate average position of robots (-pi to pi)

    for w in contending:
        cpos += (w.pos - main.ball().pos).angle()

    if len(contending) == 0:
        avgpos = 0
    else:
        avgpos = cpos / len(contending)

    return contending, avgpos
