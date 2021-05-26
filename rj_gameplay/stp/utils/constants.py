# import robocup
import stp.rc as rc
import math

OUR_CHIP_ROLL = 3  # Average distance at which a ball is slow enough to capture
THEIR_CHIPPING = (.1, .8)

class OurChipping:
    MIN_CARRY = .1
    MAX_CARRY = .8
    CAPTURE_DISTANCE = 3
    MIN_CARRY_TIME = .1
    MAX_CARRY_TIME = .4
    CAPTURE_DISTANCE_TIME = 1.2

class Colors:
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    GREEN = (0, 255, 0)
    RED = (255, 0, 0)
    BLUE = (0, 0, 255)


class Robot:
    RADIUS = 0.09
    MAX_KICK_SPEED = 8  # m/s
    CHIP_CLEARANCE = (
        .1, .6)  # min and max distance a chip will go over another robot

    class Dribbler:
        MAX_POWER = 127

        # "Normal" Dribbler speed to be used for generic ball capture/movement
        STANDARD_POWER = 127


class Ball:
    RADIUS = 0.0215
    MASS = 0.04593  # mass of golf ball (kg)

class Evaluation:
    SLOW_THRESHOLD = 1

# for field dimensions, use world_state.field (see /stp/rc.py for details)
