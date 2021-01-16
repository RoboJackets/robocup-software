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
            visible: bool, ball_sense_triggered: bool,
            has_ball_sense: bool, kicker_charged: bool,
            kicker_healthy: bool, lethal_fault: bool):

        """
        :param robot_id: Shell id of the robot.
        :param is_ours: Whether the robot is one of our robots
        :param pose: Pose of the Robot. [x, y, theta].
        :param twist: Twist of the robot. [dx, dy, dtheta].
        :param ball_sense_triggered: Whether the ball sensor is triggered
        :param visible: Whether the robot is being seen by the global vision system
        :param has_ball_sense: Whether the robots ball sensor is functional
        :param kicker_charged: Whether the robots kicker capacitors are charged
        :param kicker_healthy: Whether the robots kicker is "healthy"
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

    @classmethod
    def generate_basic_test_robot(cls, robot_id: RobotId, is_ours: bool = True,
            pose: np.ndarray = np.array([0.0, 0.0, 0.0]), 
            twist: np.ndarray = np.array([0.0, 0.0, 0.0])) -> Robot:
        """
        Returns a robot with default options for use in testing
        """
        bot = cls(robot_id, is_ours, pose, twist, ball_sense_triggered = False, visible = True,
                has_ball_sense = True, kicker_charged = True, kicker_healthy = True, lethal_fault = False)
        return bot

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
        if(not self.visible):
            #I could see removing this as it's a thing that may happen fairly often
            warnings.warn("Attempting to retrieve robot pose from non-visible robot", RuntimeWarning)

        return self.__pose

    @property
    def twist(self) -> np.ndarray:
        """
        :return: Twist of the robot. [dx, dy, dtheta].
        """
        if(not self.visible):
            #I could see removing this as it's a thing that may happen fairly often
            warnings.warn("Attempting to retrieve robot pose from non-visible robot", RuntimeWarning)

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
        """
        :param pos: ball position np.ndarray([x, y])
        :param vel: ball velocity np.ndarray([x, y])
        :param visible: True if the ball is visible
        """
        self.__pos = pos
        self.__vel = vel
        self.__visible = visible

    @classmethod
    def generate_test_ball(cls, pos: np.ndarray = np.array([0.0,0.0]), vel: np.ndarray([0.0,0.0]), visible: bool = True) -> Ball:
        ball = cls(pos, vel, visible)
        return ball

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

    __slots__ =  ["__length_m","__width_m","__border_m","__line_width_m",
            "__goal_width_m","__goal_depth_m","__goal_height_m",
            "__penalty_short_dist_m","__penalty_long_dist_m",
            "__center_radius_m","__center_diameter_m","__goal_flat_m",
            "__floor_length_m","__floor_width_m"]

    __length_m: float
    __width_m: float
    __border_m: float
    __line_width_m: float
    __goal_width_m: float
    __goal_depth_m: float
    __goal_height_m: float
    __penalty_short_dist_m: float
    __penalty_long_dist_m: float
    __center_radius_m: float
    __center_diameter_m: float
    __goal_flat_m: float
    __floor_length_m: float
    __floor_width_m: float

    def __init__(self, length_m: float, width_m: float, border_m: float,
            line_width_m: float, goal_width_m: float, goal_depth_m: float,
            goal_height_m: float, penalty_short_dist_m: float,
            penalty_long_dist_m: float, center_radius_m: float,
            center_diameter_m: float, goal_flat_m: float,
            floor_length_m: float, floor_width_m: float):
        self.__length_m = length_m
        self.__width_m = width_m
        self.__border_m = border_m
        self.__line_width_m = line_width_m
        self.__goal_width_m = goal_width_m
        self.__goal_depth_m = goal_depth_m
        self.__goal_height_m = goal_height_m
        self.__penalty_short_dist_m = penalty_short_dist_m
        self.__penalty_long_dist_m = penalty_long_dist_m
        self.__center_radius_m = center_radius_m
        self.__center_diameter_m = center_diameter_m
        self.__goal_flat_m = goal_flat_m
        self.__floor_length_m = floor_length_m
        self.__floor_width_m = floor_width_m

    @classmethod
    def generate_divA_field(cls) -> Field:
        """
        Generate a division A field

        Penalty distances and "goal_flat" need to be fixed
        """
        field = cls(length_m = 12.0, width_m = 9.0, border_m = 0.3, line_width_m = 0.01, goal_width_m = 1.8, goal_depth_m = 0.18, goal_height_m = 0.16, penalty_short_dist_m = float('nan'), penalty_long_dist_m = float('nan'), center_radius_m = 0.5, center_diameter_m = 1.0, goal_flat_m = float('nan'), floor_length_m: 13.4, floor_width_m = 10.04):
        return field

    @classmethod
    def generate_divB_field(cls) -> Field:
        """
        Generate a division B field

        Penalty distances and "goal_flat" need to be fixed
        """
        field = cls(length_m = 9.0, width_m = 6.0, border_m = 0.3, line_width_m = 0.01, goal_width_m = 1.0, goal_depth_m = 0.18, goal_height_m = 0.16, penalty_short_dist_m = float('nan'), penalty_long_dist_m = float('nan'), center_radius_m = 0.5, center_diameter_m = 1.0, goal_flat_m = float('nan'), floor_length_m: 10.04, floor_width_m = 7.4):
        return field

    @classmethod
    def generate_our_field(cls) -> Field:
        """
        Generates the practice field that we have
        """
        raise NotImplementedError("Our field specifications need to be entered")
        #field = cls(length_m = 9.0, width_m = 6.0, border_m = 0.3, line_width_m = 0.01, goal_width_m = 1.0, goal_depth_m = 0.18, goal_height_m = 0.16, penalty_short_dist_m = float('nan'), penalty_long_dist_m = float('nan'), center_radius_m = 0.5, center_diameter_m = 1.0, goal_flat_m = float('nan'), floor_length_m: 10.04, floor_width_m = 7.4):
        #return field


    @property
    def our_goal_loc(self) -> np.ndarray:
        """
        Conveniance function for getting our goal location
        :return: the location of our goal - its always (0,0)
        """
        return np.array([0.0,0.0])

    @property
    def center_field_loc(self) -> np.ndarray:
        """
        Conveniance function for getting the center field location
        :return: the location of the center of the field
        """
        return np.array([0.0, self.length_m / 2])

    @property
    def their_goal_loc(self) -> np.ndarray:
        """
        Conveniance function for getting the opponents field location
        :return: the location of the opponents goal
        """
        return np.array([0.0, self.length_m])

    @property
    def floor_width_m(self) -> float:
        """
        :return: check on this one
        """
        return self.__floor_width_m


    @property
    def floor_length_m(self) -> float:
        """
        :return: check on this one
        """
        return self.__floor_length_m

    @property
    def goal_flat_m(self) -> float:
        """
        :return: check on this one
        """
        return self.__goal_flat_m

    @property
    def center_diameter_m(self) -> float:
        """
        :return: returns the diameter of the center of the field
        """
        return self.__center_diameter_m

    @property
    def center_radius_m(self) -> float:
        """
        :return: returns the radius of the center of the field
        """
        return self.__center_radius_m

    @property
    def penalty_long_dist_m(self) -> float:
        """
        :return: double check on this one
        """
        return self.__penalty_long_dist_m

    @property
    def penalty_long_dist_m(self) -> float:
        """
        :return: double check on this one
        """
        return self.__penalty_long_dist_m

    @property
    def penalty_short_dist_m(self) -> float:
        """
        :return: double check on this one
        """
        return self.__penalty_short_dist_m

    @property
    def border_m(self) -> float:
        """
        :return: The size of the border of the field
        """
        return self.__border_m

    @property
    def line_width_m(self) -> float:
        """
        :return: The width of the lines of the field
        """
        return self.__line_width_m

    @property
    def length_m(self) -> float:
        """
        :return: The length of the field in meters
        """
        return self.__length_m

    @property
    def width_m(self) -> float:
        """
        :return: the width of the field in meters
        """
        return self.__width_m

    @property
    def goal_width_m(self) -> float:
        """
        :return: the width of the goals in meters
        """
        return self.__goal_width_m

    @property
    def goal_depth_m(self) -> float:
        """
        :return: the depth of the goals in meters
        """
        return self.__goal_depth_m

    @property
    def goal_height_m(self) -> float:
        """
        :return: the height of the goals in meters
        """
        return self.__goal_height_m


class GameInfo:
    """State of the soccer game"""

    __slots__ = ["__period", "__state", "__restart", "__our_restart"]

    __period: GamePeriod
    __state: GameState
    __restart: GameRestart
    __our_restart: bool

    def __init__(self, period: GamePeriod, state: GameState, restart: GameRestart, our_restart: bool):
        self.__period = period
        self.__state = state
        self.__restart = restart
        self.__our_restart = our_restart

    @classmethod
    def generate_test_playing_gameinfo(cls) -> GameInfo: 
        info = cls(GamePeriod.FIRST_HALF, GameState.PLAYING, GameRestart.NONE, False)
        return info

    @property
    def period(self) -> GamePeriod:
        """
        :return: The game period
        """
        return self.__period

    @property
    def state(self) -> GameState:
        """
        :return: The game state
        """
        return self.__state

    @property
    def restart(self) -> GameRestart:
        """
        :return: The game restart state
        """
        return self.__restart

    @property
    def our_restart(self) -> bool:
        """
        :return: True if it is our restart
        """
        return self.__our_restart

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

    __slots__ = ["__our_robots", "__their_robots", "__ball", "__game_info", "__field"]

    __our_robots: List[Robot]
    __their_robots: List[Robot]
    __ball: Ball
    __game_info: GameInfo
    __field: Field

    def __init__(self, our_robots: List[Robot], their_robots: List[Robot], ball: Ball, game_info: GameInfo, field: Field):
        self.__our_robots = our_robots
        self.__their_robots = their_robots
        self.__ball = ball
        self.__game_info = game_info
        self.__field = field

    @classmethod
    def generate_basic_test_worldstate(cls, our_robots = None, their_robots = None, ball = None,
            game_info = GameInfo.generate_test_playing_gameinfo(), field = Field.generate_divB_field()) -> WorldState:
        """
        generates a test worldstate with 12 robots and a ball in the play state
        """
        center_field = field.center_field_loc()
        if(our_robots is None or their_robots is None):
            our_bots = list()
            their_bots = list()
            for g in range(1,7):
                our_bots.append(Robot.generate_basic_test_robot(robot_id = g, pose = np.array(center_field + [g*0.1 - 1.0, 1.0]),is_ours = True))
                their_bots.append(Robot.generate_basic_test_robot(robot_id = g, pose = np.array(center_field + [g*0.1 - 1.0, -1.0]),is_ours = False))
        else: 
            our_bots = our_robots
            their_bots = their_robots

        if(ball is None):
            ball = Ball.generate_test_ball(pos = np.array(center_field))

        world = cls(our_bots, their_bots, ball, game_info, field)
        return world

    @property
    def robots(self) -> List[Robot]:
        """
        :return: A list of all robots (created by merging our_robots with their_robots)
        """
        return self.our_robots + self.their_robots

    @property
    def our_robots(self) -> List[Robot]:
        """
        :return: A list of our robots
        """
        return self.__our_robots

    @property
    def their_robots(self) -> List[Robot]:
        """
        :return: A list of their robots
        """
        return self.__their_robots

    @property
    def ball(self) -> Ball:
        """
        :return: The ball
        """
        return self.__ball

    @property
    def game_info(self) -> GameInfo:
        """
        :return: The GameInfo object
        """
        return self.__game_info

    @property
    def field(self) -> Field:
        """
        :return: The Field object
        """
        return self.__field


