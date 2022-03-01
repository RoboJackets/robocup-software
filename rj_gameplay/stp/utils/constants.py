# import robocup
import stp.rc as rc
import math
import sys

# for degrees->rad or reverse, use np.deg2rad or np.rad2deg)
OUR_CHIP_ROLL = 3  # Average distance at which a ball is slow enough to capture
THEIR_CHIPPING = (0.1, 0.8)


class OurChippingConstants:
    MIN_CARRY = 0.1
    MAX_CARRY = 0.8
    CAPTURE_DISTANCE = 3
    MIN_CARRY_TIME = 0.1
    MAX_CARRY_TIME = 0.4
    CAPTURE_DISTANCE_TIME = 1.2


class ColorConstants:
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    GREEN = (0, 255, 0)
    RED = (255, 0, 0)
    BLUE = (0, 0, 255)


class RobotConstants:
    RADIUS = 0.09
    MAX_KICK_SPEED = 8  # m/s
    CHIP_CLEARANCE = (
        0.1,
        0.6,
    )  # min and max distance a chip will go over another robot
    NUM_ROBOTS = 16


class DribblerConstants:
    MAX_POWER = 127

    # "Normal" Dribbler speed to be used for generic ball capture/movement
    STANDARD_POWER = 127


class BallConstants:
    RADIUS = 0.0215
    MASS = 0.04593  # mass of golf ball (kg)


class EvaluationConstants:
    SLOW_THRESHOLD = 1
    SETTLE_BALL_SPEED_THRESHOLD = 1.0
    INVALID_COST = sys.maxsize


class KickConstants:
    KICK_DOT_THRESHOLD = 0.4
    KICK_BALL_SPEED_THRESHOLD = 0.9
    MAX_KICK_SPEED = 5.0


class GoalieConstants:
    MIN_WALL_RAD = 0
    GOALIE_PCT_TO_BALL = 0.15
    DIST_TO_FAST_KICK = 7


class SituationConstants:
    POSSESS_MIN_DIST = 0.15
    MIN_PASS_SPEED = 0.9
    MIN_NEAR_BALL_DIST = 0.35


class StrikerConstants:
    OPPONENT_SPEED = 1.5
    KICK_SPEED = 4.5
    EFF_BLOCK_WIDTH = 0.7


# for field dimensions, use world_state.field (see /stp/rc.py for details)
