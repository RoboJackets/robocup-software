"""This module contains data structures that are robocup specific, ie. Robot, Ball,
WorldState"""

import warnings
from enum import Enum
from typing import List, Optional

import numpy as np

RobotId = Optional[int]


class Robot:
    """State of a robot. Pose: [x, y, theta]. Twist: [dx, dy, dtheta]. Properties are
    to enforce that instances of this class should not be mutated."""

    __slots__ = [
        "__id",
        "__is_ours",
        "__pose",
        "__twist",
        "__visible",
        "__has_ball_sense",
        "__kicker_charged",
        "__kicker_healthy",
        "__lethal_fault",
    ]

    __id: RobotId
    __is_ours: bool
    __pose: np.ndarray
    __twist: np.ndarray
    __visible: bool
    __has_ball_sense: bool
    __kicker_charged: bool
    __kicker_healthy: bool
    __lethal_fault: bool

    def __init__(
        self,
        robot_id: RobotId,
        is_ours: bool,
        pose: np.ndarray,
        twist: np.ndarray,
        visible: bool,
        has_ball_sense: bool,
        kicker_charged: bool,
        kicker_healthy: bool,
        lethal_fault: bool,
    ):
        """
        :param robot_id: Shell id of the robot.
        :param is_ours: Whether the robot is one of our robots
        :param pose: Pose of the Robot. [x, y, theta].
        :param twist: Twist of the robot. [dx, dy, dtheta].
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
        self.__visible = visible
        self.__has_ball_sense = has_ball_sense
        self.__kicker_charged = kicker_charged
        self.__kicker_healthy = kicker_healthy
        self.__lethal_fault = lethal_fault

    def __repr__(self) -> str:
        return "Robot(id:{}, is_ours:{}, pose:{}, twist:{}, visible:{})".format(
            self.__id,
            self.__is_ours,
            self.__pose,
            self.__twist,
            self.__visible,
        )

    def __eq__(self, other) -> bool:
        if isinstance(other, Robot):
            if self.is_ours == other.is_ours and self.id == other.id:
                return True
        return False

    def __hash__(self) -> int:
        return hash((self.__id, self.__is_ours))

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
        if not self.visible:
            # I could see removing this as it's a thing that may happen fairly often
            warnings.warn(
                "Attempting to retrieve robot pose from non-visible robot",
                RuntimeWarning,
            )

        return self.__pose

    @property
    def twist(self) -> np.ndarray:
        """
        :return: Twist of the robot. [dx, dy, dtheta].
        """
        if not self.visible:
            # I could see removing this as it's a thing that may happen fairly often
            warnings.warn(
                "Attempting to retrieve robot pose from non-visible robot",
                RuntimeWarning,
            )

        return self.__twist

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
        if not self.is_ours:
            warnings.warn(
                "Attempting to retrieve ball sense status from an opposing robot",
                RuntimeWarning,
            )

        return self.__has_ball_sense

    @property
    def kicker_charged(self) -> bool:
        """
        :return: True if the kicker capacitors are charged
        """
        if not self.is_ours:
            warnings.warn(
                "Attempting to retrieve kicker charge status from an opposing robot",
                RuntimeWarning,
            )
            return False

        return self.__kicker_charged

    @property
    def kicker_healthy(self) -> bool:
        """
        :return: True if the kicker is healthy
        """
        if not self.is_ours:
            warnings.warn(
                "Attempting to retrieve kicker health status from an opposing robot",
                RuntimeWarning,
            )
            return False

        return self.__kicker_healthy

    @property
    def lethal_fault(self) -> bool:
        """
        :return: True if the robot has encounted a fault that will prevent further play, such as an FPGA or motor fault.
        """
        if not self.is_ours:
            warnings.warn(
                "Attempting to retrieve lethal fault information from an opposing"
                " robot",
                RuntimeWarning,
            )

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

    def __repr__(self) -> str:
        return "Ball(pos:{}, vel:{}, visible:{})".format(
            self.__pos, self.__vel, self.__visible
        )

    @property
    def pos(self) -> np.ndarray:
        """
        :return: Position of the ball. [x, y].
        """
        if not self.visible:
            warnings.warn(
                "Retrieved the position of a non-visible ball", RuntimeWarning
            )

        return self.__pos

    @property
    def vel(self) -> np.ndarray:
        """
        :return: Velocity of the ball. [dx, dy].
        """
        if not self.visible:
            warnings.warn(
                "Retrieved the velocity of a non-visible ball", RuntimeWarning
            )

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
    PENALTY_PLAYING = 5  # All robots except the striker and the goalie must stay on the opposite side of the field.


class GameRestart(Enum):
    """What kind of restart."""

    NONE = 0
    KICKOFF = 1
    FREE = 2
    PENALTY = 3
    PLACEMENT = 4


class Field:
    """Information about the field."""

    __slots__ = [
        "__length_m",
        "__width_m",
        "__border_m",
        "__line_width_m",
        "__goal_width_m",
        "__goal_depth_m",
        "__goal_height_m",
        "__def_area_short_dist_m",
        "__def_area_long_dist_m",
        "__center_radius_m",
        "__center_diameter_m",
        "__goal_flat_m",
        "__floor_length_m",
        "__floor_width_m",
        "__def_area_x_right_coord",
        "__def_area_x_left_coord",
        "__field_x_right_coord",
        "__field_x_left_coord",
    ]

    __length_m: float
    __width_m: float
    __border_m: float
    __line_width_m: float
    __goal_width_m: float
    __goal_depth_m: float
    __goal_height_m: float
    __def_area_short_dist_m: float
    __def_area_long_dist_m: float
    __center_radius_m: float
    __center_diameter_m: float
    __goal_flat_m: float
    __floor_length_m: float
    __floor_width_m: float
    __def_area_x_right_coord: float
    __def_area_x_left_coord: float
    __field_x_right_coord: float
    __field_x_left_coord: float

    def __init__(
        self,
        length_m: float,
        width_m: float,
        border_m: float,
        line_width_m: float,
        goal_width_m: float,
        goal_depth_m: float,
        goal_height_m: float,
        def_area_short_dist_m: float,
        def_area_long_dist_m: float,
        center_radius_m: float,
        center_diameter_m: float,
        goal_flat_m: float,
        floor_length_m: float,
        floor_width_m: float,
    ):
        self.__length_m = length_m
        self.__width_m = width_m
        self.__border_m = border_m
        self.__line_width_m = line_width_m
        self.__goal_width_m = goal_width_m
        self.__goal_depth_m = goal_depth_m
        self.__goal_height_m = goal_height_m
        self.__def_area_short_dist_m = def_area_short_dist_m
        self.__def_area_long_dist_m = def_area_long_dist_m
        self.__center_radius_m = center_radius_m
        self.__center_diameter_m = center_diameter_m
        self.__goal_flat_m = goal_flat_m
        self.__floor_length_m = floor_length_m
        self.__floor_width_m = floor_width_m
        self.__def_area_x_right_coord = def_area_long_dist_m / 2
        self.__def_area_x_left_coord = -(def_area_long_dist_m / 2)
        self.__field_x_right_coord = width_m / 2
        self.__field_x_left_coord = -(width_m / 2)

    @property
    def our_goal_loc(self) -> np.ndarray:
        """
        Convenience function for getting our goal location
        :return: the location of our goal - its always (0,0)
        """
        return np.array([0.0, 0.0])

    @property
    def center_field_loc(self) -> np.ndarray:
        """
        Convenience function for getting the center field location
        :return: the location of the center of the field
        """
        return np.array([0.0, self.length_m / 2])

    @property
    def their_goal_loc(self) -> np.ndarray:
        """
        Convenience function for getting the opponents field location
        :return: the location of the opponents goal
        """
        return np.array([0.0, self.length_m])

    @property
    def our_defense_area_coordinates(self) -> List:
        """
        Convenience function for getting our defense area locations
        :return: the list of points for our defense area locations
        """
        our_defense_area = [
            [self.__def_area_x_left_coord, self.__def_area_short_dist_m],
            [self.__def_area_x_right_coord, self.__def_area_short_dist_m],
            [self.__def_area_x_left_coord, 0.0],
            [self.__def_area_x_right_coord, 0.0],
        ]
        return our_defense_area

    @property
    def opp_defense_area_coordinates(self) -> List:
        """
        Convenience function for getting oppenent defense area locations
        Note: each coordinate starts from top left and continues normal order
        :return: the list of points for opponent defense area locations
        """
        opp_defense_area = [
            [self.__def_area_x_left_coord, self.__length_m],
            [self.__def_area_x_right_coord, self.__length_m],
            [
                self.__def_area_x_left_coord,
                self.__length_m - self.__def_area_short_dist_m,
            ],
            [
                self.__def_area_x_right_coord,
                self.__length_m - self.__def_area_short_dist_m,
            ],
        ]
        return opp_defense_area

    @property
    def our_goal_post_coordinates(self) -> List:
        """
        Convenience function for getting our goal post coordinates
        :return: the list of points for our goal post locations
        """
        our_goal_post_coord = [
            [-self.__goal_width_m / 2, 0],
            [self.__goal_width_m / 2, 0],
        ]
        return our_goal_post_coord

    @property
    def their_goal_post_coordinates(self) -> List:
        """
        Convenience function for getting their goal post coordinates
        :return: the list of points for their goal post locations
        """
        their_goal_post_coord = [
            [-self.__goal_width_m / 2, self.__length_m],
            [self.__goal_width_m / 2, self.__length_m],
        ]
        return their_goal_post_coord

    @property
    def our_left_corner(self) -> np.ndarray:
        """
        :return: the coords of the left corner of our side of the field
        """
        return np.array([self.__field_x_left_coord, 0.0])

    @property
    def our_right_corner(self) -> np.ndarray:
        """
        :return: the coords of the right corner of our side of the field
        """
        return np.array([self.__field_x_right_coord, 0.0])

    @property
    def their_left_corner(self) -> np.ndarray:
        """
        :return: the coords of the left corner of their side of the field
        """
        return np.array([self.__field_x_left_coord, self.__length_m])

    @property
    def their_right_corner(self) -> np.ndarray:
        """
        :return: the coords of the right corner of their side of the field
        """
        return np.array([self.__field_x_right_coord, self.__length_m])

    @property
    def floor_width_m(self) -> float:
        """
        :return: width of full field (including borders)
        """
        return self.__width_m + 2 * self.__border_m

    @property
    def def_area_x_left_coord(self) -> float:
        """
        :return: left x coordinate of the defense area
        """
        return self.__def_area_x_left_coord

    @property
    def def_area_x_right_coord(self) -> float:
        """
        :return: right x coordinate of the defense area
        """
        return self.__def_area_x_right_coord

    @property
    def floor_length_m(self) -> float:
        """
        :return: length of full field (including borders)
        """
        return self.__length_m + 2 * self.__border_m

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
    def def_area_long_dist_m(self) -> float:
        """
        :return: double check on this one
        """
        return self.__def_area_long_dist_m

    @property
    def def_area_short_dist_m(self) -> float:
        """
        :return: double check on this one
        """
        return self.__def_area_short_dist_m

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
    """State of the soccer game. Corresponds to a combination of the C++-side PlayState and MatchState"""

    __slots__ = [
        "__period",
        "__state",
        "__restart",
        "__our_restart",
        "__ball_placement",
    ]

    __period: GamePeriod
    __state: GameState
    __restart: GameRestart
    __our_restart: bool
    __ball_placement: np.array

    def __init__(
        self,
        period: GamePeriod,
        state: GameState,
        restart: GameRestart,
        our_restart: bool,
        ball_placement: np.array,
    ):
        self.__period = period
        self.__state = state
        self.__restart = restart
        self.__our_restart = our_restart
        self.__ball_placement = ball_placement

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
        if not self.is_restart():
            warnings.warn(
                "Retrieved if it is our restart when it is not a restart at all",
                RuntimeWarning,
            )
            return False  # Is returning this dangerous?

        return self.__our_restart

    @property
    def their_restart(self) -> bool:
        """
        :return: True if it is their restart
        """
        if not self.is_restart():
            warnings.warn(
                "Retrieved if it is our restart when it is not a restart at all",
                RuntimeWarning,
            )
            return False  # Is returning this dangerous?

        return not self.__our_restart

    def is_stopped(self) -> bool:
        """
        :return: True if play is stopped.
        """
        return self.state == GameState.STOP

    def is_ready(self) -> bool:
        """
        :return: True if the field is waiting on a team to kick the ball in a restart.
        """
        return self.state == GameState.READY

    def is_setup(self) -> bool:
        """
        :return: True if the field is setting up for a penalty kick or kickoff.
        """
        return self.state == GameState.SETUP

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

    def is_free(self) -> bool:
        """
        :return: True if the restart is a free kick.
        """
        return self.restart == GameRestart.FREE

    def is_free_placement(self) -> bool:
        """
        :return: True if the restart is free placement.
        """
        return self.restart == GameRestart.PLACEMENT

    def ball_placement(self) -> Optional[np.ndarray]:
        """
        :return: True if the restart is free placement.
        """
        return self.__ball_placement if self.is_free_placement() else None


class WorldState:
    """Current state of the world."""

    __slots__ = [
        "__our_robots",
        "__their_robots",
        "__ball",
        "__game_info",
        "__field",
        "__goalie_id",
    ]

    __our_robots: List[Robot]
    __their_robots: List[Robot]
    __ball: Ball
    __game_info: GameInfo
    __field: Field
    __goalie_id: int

    def __init__(
        self,
        our_robots: List[Robot],
        their_robots: List[Robot],
        ball: Ball,
        game_info: GameInfo,
        field: Field,
        goalie_id: int,
    ):
        self.__our_robots = our_robots
        self.__their_robots = their_robots
        self.__ball = ball
        self.__game_info = game_info
        self.__field = field
        self.__goalie_id = goalie_id

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

    @property
    def goalie_id(self) -> int:
        """
        :return: The goalie id (int)
        """
        return self.__goalie_id

    @property
    def visible_robots(self) -> List[Robot]:
        """
        :return: List of all visible robots
        """
        return [robot for robot in self.robots if robot.visible]

    @property
    def our_visible_robots(self) -> List[Robot]:
        """
        :return: List of all our visible robots
        """
        return [robot for robot in self.our_robots if robot.visible]

    @property
    def their_visible_robots(self) -> List[Robot]:
        """
        :return: List of all their visible robots
        """
        return [robot for robot in self.their_robots if robot.visible]
