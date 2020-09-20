"""This module contains data structures that are robocup specific, ie. Robot, Ball,
WorldState"""

from enum import Enum
from typing import List

import numpy as np

RobotId = int


class Robot:
    """Robot. Pose: [x, y, theta]. Twist: [dx, dy, dtheta]."""

    __slots__ = ["id", "pose", "twist", "has_ball"]
    id: RobotId
    pose: np.ndarray
    twist: np.ndarray
    has_ball: bool

    def __init__(
        self, robot_id: RobotId, pose: np.ndarray, twist: np.ndarray, has_ball: bool
    ):
        """
        :param robot_id: Shell id of the robot.
        :param pose: Pose of the Robot. [x, y, theta].
        :param twist: Twist of the robot. [dx, dy, dtheta].
        :param has_ball: Whether the robot has the ball or not.
        """
        self.id = robot_id
        self.pose = pose
        self.twist = twist
        self.has_ball = has_ball

    def __repr__(self) -> str:
        return "Robot(id:{}, pose:{}, twist:{}, has_ball:{})".format(
            self.id, self.pose, self.twist, self.has_ball
        )


class Ball:
    """Ball."""

    __slots__ = ["pos", "vel"]

    pos: np.ndarray
    vel: np.ndarray

    def __init__(self, pos: np.ndarray, vel: np.ndarray):
        self.pos = pos
        self.vel = vel

    def __repr__(self) -> str:
        return "Ball(pos:{}, vel:{})".format(self.pos, self.vel)


class GamePeriod(Enum):
    FIRST_HALF = 0
    HALF_TIME = 1
    SECOND_HALF = 2
    OVERTIME1 = 3
    OVERTIME2 = 4
    PENALTY_SHOOTOUT = 5


class GameState(Enum):
    HALT = 0  # Robots must not move.
    STOP = 1  # Robots must stay 500mm away from the ball.
    SETUP = 2  # Robots not on starting team msut stay 500mm away from ball.
    READY = 3  # A robot on the starting team may kick the ball.
    PLAYING = 4  # Normal play.


class GameRestart(Enum):
    NONE = 0
    KICKOFF = 1
    DIRECT = 2
    INDIRECT = 3
    PENALTY = 4
    PLACEMENT = 5


class Field:
    """Information about the field."""

    __slots__ = ["length_m"]

    length_m: float

    def __init__(self, length_m: float):
        self.length_m = length_m


class GameInfo:
    """Game state."""

    __slots__ = ["period", "state", "restart", "our_restart", "field"]

    period: GamePeriod
    state: GameState
    restart: GameRestart
    our_restart: bool
    field: Field

    def is_restart(self) -> bool:
        return self.restart != GameRestart.NONE

    def is_kickoff(self) -> bool:
        return self.restart == GameRestart.KICKOFF

    def is_penalty(self) -> bool:
        return self.restart == GameRestart.PENALTY

    def is_direct(self) -> bool:
        return self.restart == GameRestart.DIRECT

    def is_indirect(self) -> bool:
        return self.restart == GameRestart.INDIRECT

    def is_free_placement(self) -> bool:
        return self.restart == GameRestart.PLACEMENT


class WorldState:
    """Current state of the game."""

    __slots__ = ["our_robots", "their_robots", "ball"]

    def __init__(self, our_robots: List[Robot], their_robots: List[Robot], ball: Ball):
        self.our_robots = our_robots
        self.their_robots = their_robots
        self.ball = ball
