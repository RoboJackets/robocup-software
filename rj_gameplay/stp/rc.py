"""This module contains data structures that are robocup specific, ie. Robot, Ball,
WorldState"""

from enum import Enum
from typing import List

import numpy as np
import warnings

RobotId = int


class Robot:
    """State of a robot. Pose: [x, y, theta]. Twist: [dx, dy, dtheta]. Properties are
    to enforce that instances of this class should not be mutated."""

    __slots__ = ["__id", "__is_ours", "__pose", "__twist", "__ball_sense_triggered",
            "__visible", "__has_ball_sense", "__kicker_charged", "__kicker_healthy", "__lethal_fault"]
    __id: RobotId
    __is_ours: bool
    __pose: np.ndarray
    __twist: np.ndarray
    __visible: bool
    __ball_sense_triggered: bool
    __has_ball_sense: bool
    __kicker_charged: bool
    __kicker_healthy: bool
    __lethal_fault: bool
    

    def __init__(
            self, robot_id: RobotId, is_ours: bool, pose: np.ndarray, twist: np.ndarray,
            visible = True: bool, ball_sense_triggered = False: bool,
            has_ball_sense = True: bool, kicker_charged = True: bool,
            kicker_healthy = True: bool, lethal_fault = False: bool):
        """
        :param robot_id: Shell id of the robot.
        :param is_ours: Whether the robot is one of our robots
        :param pose: Pose of the Robot. [x, y, theta].
        :param twist: Twist of the robot. [dx, dy, dtheta].
        :param ball_sense_triggered: Whether the ball sensor is triggered
        :param visible: Whether the robot is being seen by the global vision system
        :param has_ball_sense: Whether the robots ball sensor is functional
        :param kicker_charged: Whether the robots kicker capacitors are charged
        :param kicker_healthy: Whether the robots kicker is "healthy" (check on what that means)
        :param lethal_fault: Whether the robot has experienced a fault that will prevent it from functioning
        """
        self.__id = robot_id
        self.__is_ours = is_ours
        self.__pose = pose
        self.__twist = twist
        self.__ball_sense_triggered = ball_sense_triggered
        self.__visible = visible
        self.__has_ball_sense = has_ball_sense
        self.__kicker_charged = kicker_charged
        self.__kicker_healthy = kicker_healthy
        self.__lethal_fault = lethal_fault

    def __repr__(self) -> str:
        return "Robot(id:{}, is_ours:{}, pose:{}, twist:{}, visible:{})".format(
            self.__id, self.__is_ours, self.__pose, self.__twist, self.__visible)

    def __eq__(self, other) -> bool:
        if isinstance(other, Robot):
            if(self.is_ours == other.is_ours and self.id == other.id):
                return True
        return False 

    @property
    def id(self) -> RobotId:
        """
        :return: Id of the robot.
        """
        return self.__id

    @property
    def is_ours(self) -> bool:
        """
        :return: True if the robot is our robot.
        """
        return self.__is_ours

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
    def ball_sense_triggered(self) -> bool:
        """
        :return: True if the ball sense break-beam is triggered
        """
        if(not self.is_ours):
            warnings.warn("Attempting to retrieve ball sense information from an opposing robot", RuntimeWarning)
            return False

        return self.__ball_sense_triggered

    @property
    def visible(self) -> bool:
        """
        :return: True if the robot is visible
        """
        return self.__visible

    @property
    def has_ball_sense(self) -> bool:
        """
        :return: True if this robot has functioning ball sensors
        """
        if(not self.is_ours):
            warnings.warn("Attempting to retrieve ball sense status from an opposing robot", RuntimeWarning)

        return self.__has_ball_sense

    @property
    def kicker_charged(self) -> bool:
        """
        :return: True if the kicker capacitors are charged
        """
        if(not self.is_ours):
            warnings.warn("Attempting to retrieve kicker charge status from an opposing robot", RuntimeWarning)
            return False

        return self.__kicker_charged

    @property
    def kicker_healthy(self) -> bool:
        """
        :return: True if the kicker is healthy
        """
        if(not self.is_ours):
            warnings.warn("Attempting to retrieve kicker health status from an opposing robot", RuntimeWarning)
            return False

        return self.__kicker_healthy


    @property
    def lethal_fault(self) -> bool:
        """
        :return: True if the robot has encounted a fault that will prevent further play, such as an FPGA or motor fault.
        """
        if(not self.is_ours):
            warnings.warn("Attempting to retrieve lethal fault information from an opposing robot", RuntimeWarning)

        return self.__lethal_fault


class Ball:
    """State of the ball. Properties are used to enforce that instances of this class
    should not be mutated."""

    __slots__ = ["__pos", "__vel", "__visible"]

    __pos: np.ndarray
    __vel: np.ndarray
    __visible: bool

    def __init__(self, pos: np.ndarray, vel: np.ndarray, visible: bool):
        self.__pos = pos
        self.__vel = vel
        self.__visible = visible

    def __repr__(self) -> str:
        return "Ball(pos:{}, vel:{}, visible:{})".format(self.__pos, self.__vel, self.__visible)

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

    @property
    def visible(self) -> bool:
        """
        :return: True if the ball can currently be seen by the global vision system.

        Note that with filtering or additional sensors like whiskers we may still know where the ball is located
        """
        return self.__visible


class GamePeriod(Enum):
    """Game period."""

    FIRST_HALF = 0
    HALF_TIME = 1
    SECOND_HALF = 2
    OVERTIME1 = 3
    OVERTIME2 = 4
    PENALTY_SHOOTOUT = 5


class GameState(Enum):
    """State of the game."""

    HALT = 0  # Robots must not move.
    STOP = 1  # Robots must stay 500mm away from the ball.
    SETUP = 2  # Robots not on starting team msut stay 500mm away from ball.
    READY = 3  # A robot on the starting team may kick the ball.
    PLAYING = 4  # Normal play.


class GameRestart(Enum):
    """What kind of restart."""

    NONE = 0
    KICKOFF = 1
    DIRECT = 2
    INDIRECT = 3
    PENALTY = 4
    PLACEMENT = 5


class Field:
    """Information about the field."""

    __slots__ = ["__length_m","__width_m","__goal_width_m"]

    __length_m: float
    __width_m: float
    __goal_width_m: float

    """
    A list of the information avalible in the FieldDimensions.msg
    we probably need many of these included in WorldState
    float32 length
    float32 width
    float32 border - What is border?
    float32 line_width - Does gameplay need this? 
    float32 goal_width
    float32 goal_depth - Does gameplay need this?
    float32 goal_height - Does gameplay need this?
    float32 penalty_short_dist
    float32 penalty_long_dist
    float32 center_radius
    float32 center_diameter
    float32 goal_flat
    float32 floor_length
    float32 floor_width
    """

    def __init__(self, length_m: float, width_m: float, goal_width_m: float):
        self.length_m = length_m
        self.width_m = width_m

    @property
    def length_m(self) -> float:
        """
        :returns: The length of the field in meters
        """
        return self.__length_m

    @property
    def width_m(self) -> float:
        """
        :returns: the width of the field in meters
        """
        return self.__length_m

    @property
    def goal_width_m(self) -> float:
        """
        :returns: the width of the goals in meters
        """
        return self.__goal_width_m

    @property
    def our_goal_loc(self) -> np.ndarray:
        """
        Conveniance function for getting our goal location
        :returns: the location of our goal - its always (0,0)
        """
        return np.array([0.0,0.0])

    @property
    def center_field_loc(self) -> np.ndarray:
        """
        Conveniance function for getting the center field location
        :returns: the location of the center of the field
        """
        return np.array([0.0, self.length_m / 2])

    @property
    def their_goal_loc(self) -> np.ndarray:
        """
        Conveniance function for getting the opponents field location
        :returns: the location of the opponents goal
        """
        return np.array([0.0, self.length_m])


class GameInfo:
    """State of the soccer game"""

    __slots__ = ["period", "state", "restart", "our_restart", "field"]

    period: GamePeriod
    state: GameState
    restart: GameRestart
    our_restart: bool

    def is_restart(self) -> bool:
        """
        :return: True if there is a restart.
        """
        return self.restart != GameRestart.NONE

    def is_kickoff(self) -> bool:
        """
        :return: True if the restart is a kickoff.
        """
        return self.restart == GameRestart.KICKOFF

    def is_penalty(self) -> bool:
        """
        :return: True if the restart is a penalty.
        """
        return self.restart == GameRestart.PENALTY

    def is_direct(self) -> bool:
        """
        :return: True if the restart is a direct kick.
        """
        return self.restart == GameRestart.DIRECT

    def is_indirect(self) -> bool:
        """
        :return: True if the restart is an indirect kick.
        """
        return self.restart == GameRestart.INDIRECT

    def is_free_placement(self) -> bool:
        """
        :return: True if the restart is free placement.
        """
        return self.restart == GameRestart.PLACEMENT


class WorldState:
    """Current state of the world."""

    __slots__ = ["our_robots", "their_robots", "ball", "game_info", "field"]

    our_robots: List[Robot]
    their_robots: List[Robot]
    ball: Ball
    game_info: GameInfo
    field: Field

    def __init__(self, our_robots: List[Robot], their_robots: List[Robot], ball: Ball, game_info: GameInfo, field: Field):
        self.our_robots = our_robots
        self.their_robots = their_robots
        self.ball = ball
        self.game_info = game_info
        self.field = field

    @property
    def robots(self) -> List[Robot]:
        return self.our_robots + self.their_robots
