from enum import Enum
from lzma import is_check_supported

import stp.role
import stp.rc

from typing import List, TypedDict, Tuple

from rj_gameplay.skill import receive, pivot_kick  # , line_kick
import rj_gameplay.rj_gameplay.gameplay_node as gameplay_node

from rj_msgs.msg import RobotIntent

import numpy as np

class State(Enum):
    # Initialization
    initializing = 0
    # Robot is settling ball
    receiving = 1
    # Robot has ball and is considering options
    possessing = 2
    # Robot has identified pass and is passing
    passing = 3
    # Robot has identified shot on goal and is shooting
    shooting = 4

class BallReleaseState(Enum):
    # Robot does not plan to get rid of ball
    holding = 0
    # Robot is getting ready to pass or shoot
    initializing = 1
    # Robot is executing pass or shoot
    executing = 2
    # Robot no longer has the ball
    finalized = 3


# Constant for how far a robot can be from the pass line before stopping the pass from occuring
PASS_CUTOFF = 0.1
# Constant for how far in front of (but equally distant) a point this robot will investigate when
# checking for leading passes
THETA_LEADING_OFFSET = 0.1
# Constant for how far from the path of the ball the closest enemy robot can be
SHOOT_COVERAGE_CUTOFF = 0.1

class PasserRole(stp.role.Role):
    def __init__(self, robot: stp.rc.Robot) -> None:
        super().__init__(robot)

        self.receive_skill = None
        self.pivot_kick_skill = None

        # TODO: make FSM class (or at least use enum instead of str literals)
        self._state = State.initializing
        self._ball_state = BallReleaseState.holding

        self._target_point = None

        # BEGIN dynamic tendency variables #
        
        # floating point value [0,1] representing how open this robot is, i.e. how much coverage
        # is the other team placing on this robot and how "at risk" is the ball towards being stolen.
        # Another way of saying this is how close (in terms of path planning) is the closest robot to the ball.
        self.robot_openness = 0

        # floating point value [0,1] representing how much of the percieved goal is being covered by the
        # other teams goalie and players.  Basically, in the line of sight of this robot what percent of the goal is
        # being occupied by the opposing teams goalie (and players) + a buffer of _____%.
        self.shot_on_goal = [False, np.zeros(0, 0)]

        # a dictionary mapping robot ids to floating point values [0,1] representing how much of each teammate is being covered by the
        # other teams players.  Basically, in the line of sight of this robot what percent of the teammate is
        # being occupied by the opposing team + a buffer of ____%.
        self.teammate_direct_openness = {}

        # a dictionary mapping robot ids to floating point values [-1,1] representing how much coverege is in the counter-clockwise (-) (with this robot
        # as the pivot) of another robot and or in the positive direction (+) (again with this robot as the pivot point) dependent
        # on which direction is closer to the goal.
        self.teammate_leading_openness = []
        # END dynamic tendency variables #

    @property
    def pass_ready(self):
        return self._state == "pass_ready"

    def set_execute_pass(self, target_point):
        self._state = "init_execute_pass"
        self._target_point = target_point

    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
        """
        Assume robot does not have ball on init. Then:
         - on init: get ball
         - when got ball: mark pass ready for Tactic, dribble, wait
         - on pass signal from Tactic: pivot_kick to point, let receiver get ball, done
        """

        displacements = self.calc_displacement_vecs(world_state)
        distances = self.calc_dist_from_lines(world_state, displacements)
        self.teammate_direct_openness = self.can_pass_to_teammates(world_state, distances)

        self.shot_on_goal = self.is_shot_on_goal(world_state)

        intent = None

        if self._state == "init":
            self.receive_skill = receive.Receive(robot=self.robot)
            intent = self.receive_skill.tick(world_state)
            self._state = "receiving"
        elif self._state == "receiving":
            intent = self.receive_skill.tick(world_state)
            if self.receive_skill.is_done(world_state):
                self._state = "pass_ready"
        elif self._state == "pass_ready":
            # TODO: dribble until the receiver is ready
            pass
        # this state transition is done by the PassTactic, which is not canonical FSM
        elif self._state == "init_execute_pass":
            # TODO: make these params configurable
            self.pivot_kick_skill = pivot_kick.PivotKick(
                robot=self.robot,
                target_point=self._target_point,
                chip=False,
                kick_speed=4.0,  # TODO: adjust based on dist from target_point
            )
            self._state = "execute_pass"
        elif self._state == "execute_pass":
            intent = self.pivot_kick_skill.tick(world_state)

            if self.pivot_kick_skill.is_done(world_state):
                self._state = "kick_done"
                # end FSM

        return intent

    def is_done(self, world_state) -> bool:
        return self._state == "kick_done"

    def calc_displacement_vecs(
            self, our_robots: List[stp.rc.Robot], this_robot_id: stp.rc.RobotId, this_robot_pos: np.ndarray
        ) -> TypedDict[stp.rc.RobotId: np.ndarray]:
        """
        Calculates the displacement between all of our robots and the robot that has this role.
        
        :param our_robots: the list of our robots
        :type our_robots: List[stp.rc.Robot]
        :param this_robot_id: the id of the robot that is given the passer/ball_handler role
        :type this_robot_id: stp.rc.RobotId (int)
        :param this_robot_pos: the position of this robot in the x,y grid
        :type this_robot_pos: np.ndarray

        :return: a displacement vector between one of our robots and the robot currently assigned to the passer/ball_handler role
        :type return: TypedDict[RobotId: np.ndarray]
        """

        vectors = {}
        for i in range(0, len(our_robots)):
            if our_robots[i].id == this_robot_id:
                vectors[our_robots[i].id] = None
                continue
            vectors[our_robots[i].id] = our_robots[i].pose[0:2] - this_robot_pos
        return vectors

    def calc_dist_from_lines(
            self, our_robots: List[stp.rc.Robot], their_robots: List[stp.rc.Robot], displacement_vectors: TypedDict[stp.rc.RobotId: np.ndarray],
            this_robot_pos: np.ndarray, this_robot_id: stp.rc.RobotId
        ) -> np.array:
        """
        Calculates the distance from our displacement lines of each robot on the field.

        :param our_robots: A list of our robots
        :type our_robots: List[Robot]
        :param their_robots: A list of the other team's robots
        :type their_robots: List[Robot]

        :return list of distance of each robot (listed in order by id) from the displacement vector spanning from this robot
        :type: np.ndarray of shape [NUM_ROBOTS / 2, NUM_ROBOTS] (distance from line per robot)
        """

        distances = np.zeros(gameplay_node.NUM_ROBOTS / 2, gameplay_node.NUM_ROBOTS)
        for i in range(0, len(displacement_vectors)):
            for j in range(0, gameplay_node.NUM_ROBOTS):
                distance = np.cross(displacement_vectors[i], world_state.robots[j]) / np.linalg.norm(displacement_vectors[i])
                distances[i,j] = distance
        return distances

    def can_direct_pass_to_teammates(self, world_state: stp.rc.WorldState, distances: np.ndarray) -> TypedDict[int: bool]:
        """
        Creates a list of boolean values indicating whether or not this robot can pass to a friendly robot.  These values are
        calculated by setting a cutoff threshold that, when a distance is less than, decides that a robot is no longer open.

        :param world_state: the current world state
        :type world_state: stp.rc.WorldState
        :param distances: the numpy array of distances of each robot from the line between this robot and the designated receiver.
        :type distances: np.ndarray of shape [NUM_ROBOTS / 2, NUM_ROBOTS]

        :return boolean value indicating whether the robot is open for a direct pass
        :type return: TypedDict[int: bool] mapping robots by their id to whether or not they are open
        """

        passable_list = {}
        for i in range(0, gameplay_node.NUM_ROBOTS / 2):
            passable_list[world_state.our_robots[i].id] = True
            for j in range(0, gameplay_node.NUM_ROBOTS):
                if (world_state.robots[j] != self.robot and world_state.robot[i] != world_state.robot[j]):
                    if distances[i,j] < PASS_CUTOFF:
                        passable_list[world_state.our_robots[i]] = False
                        break
        return passable_list

    def is_shot_on_goal(self, world_state: stp.rc.WorldState) -> Tuple(bool, np.ndarray):
        """
        Checks a few points in the goal line to see if a shot at the location would be blockable by the opposing team's goalie.

        :param world_state: the current world state
        :type worl_state: stp.rc.WorldState

        :return: Tuple that says whether or not a shot can be made as well as the location the shot should be shot at.
        :type return: Tuple(bool, np.ndarray)
        """
        
        num_divisions = 5
        goal_top = np.zero(2)
        goal_top[0] = gameplay_node.GameplayNode.field.goal_width_m / 2 + physical_goal