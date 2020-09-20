"""This module contains data structures that are robocup specific, ie. Robot, Ball,
WorldState"""

from enum import Enum
from typing import List

import numpy as np

RobotId = int


class Robot:
    """State of the robot. Pose: [x, y, theta]. Twist: [dx, dy, dtheta]. Properties are
    to enforce that instances of this class should not be mutated."""

    __slots__ = ["__id", "__pose", "__twist", "__has_ball"]
    __id: RobotId
    __pose: np.ndarray
    __twist: np.ndarray
    __has_ball: bool

    def __init__(
        self, robot_id: RobotId, pose: np.ndarray, twist: np.ndarray, has_ball: bool
    ):
        """
        :param robot_id: Shell id of the robot.
        :param pose: Pose of the Robot. [x, y, theta].
        :param twist: Twist of the robot. [dx, dy, dtheta].
        :param has_ball: Whether the robot has the ball or not.
        """
        self.__id = robot_id
        self.__pose = pose
        self.__twist = twist
        self.__has_ball = has_ball

    def __repr__(self) -> str:
        return "Robot(id:{}, pose:{}, twist:{}, has_ball:{})".format(
            self.__id, self.__pose, self.__twist, self.__has_ball
        )

    @property
    def id(self) -> RobotId:
        """
        :return: Id of the robot.
        """
        return self.__id

    @property
    def pose(self) -> np.ndarray:
        """
        :return: Pose of the robot. [x, y, theta].
        """
        return self.__pose

    @property
    def twist(self) -> np.ndarray:
        """
        :return: Twist of the robot. [dx, dy, dtheta].
        """
        return self.__twist

    @property
    def has_ball(self) -> bool:
        return self.__has_ball


class Ball:
    """State of the ball. Properties are used to enforce that instances of this class
    should not be mutated."""

    __slots__ = ["__pos", "__vel"]

    __pos: np.ndarray
    __vel: np.ndarray

    def __init__(self, pos: np.ndarray, vel: np.ndarray):
        self.__pos = pos
        self.__vel = vel

    def __repr__(self) -> str:
        return "Ball(pos:{}, vel:{})".format(self.__pos, self.__vel)

    @property
    def pos(self) -> np.ndarray:
        """
        :return: Position of the ball. [x, y].
        """
        return self.__pos

    @property
    def vel(self) -> np.ndarray:
        """
        :return: Velocity of the ball. [dx, dy].
        """
        return self.__vel


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
