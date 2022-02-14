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

        self.min_rob_dist_pass = []

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

        our_vecs, their_vecs = self.calc_displacement_vecs(world_state)
        self.min_rob_dist_pass = self.calc_min_distance_from_lines(our_vecs, their_vecs)

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

    def calc_displacement_vecs(self, world_state: stp.rc.WorldState) -> Tuple[np.ndarray]:
        """
        Calculates the vector between the robot assigned the passer role and the other robots on the field.  For the case where the robot is checking against
        it's own position, the vector given is (0, 0)

        :param world_state: the world_state representation we have each tick
        :type world_state: stp.rc.WorldState

        :return [(our_robots), 3] array corresponding to the displacement vector (x, y, 0) this robot to each of our other robots and a second array between this robot
        and the other teams robots
        :type return: 2 np.ndarray of dimension [(NUM_ROBOTS / 2), 3]
        """
        our_vecs = np.zeros(gameplay_node.NUM_ROBOTS / 2, 3)
        for i in range(0, gameplay_node.NUM_ROBOTS / 2):
            if (world_state.our_robots[i].id != self.robot.id):
                our_vecs[i,0:2] = world_state.our_robots[i].pose[0:2] - self.robot.pose[0:2]
        their_vecs = np.zeros(gameplay_node.NUM_ROBOTS / 2, 2)
        for j in range(0, gameplay_node.NUM_ROBOTS / 2):
            their_vecs[i,0:2] = world_state.their_robots[i].pose[0:2] - self.robot.pose[0:2]
        return our_vecs, their_vecs

    def calc_min_distance_from_lines(self, our_vecs: np.ndarray, their_vecs: np.ndarray) -> np.ndarray:
        """
        Creates a list of the distance from the closest robot to the path the ball will have to take to make a direct pass to each teammate.

        :param our_vecs: the np array of displacement vectors from this robot to the robot with id equal to the index of the (x,y) pair in the
        numpy array
        :type our_vecs: np.ndarray of size [(our_robots),3] (x, y, 0)
        :param their_vecs: the np array of displacement vectors from this robot to the other team's robots
        :type their_vecs: np.ndarray of size [(their_robots), 3] (x, y, 0)]

        :return list of distance from robot closest to the linear path of the ball to the path the ball will have to take to make a direct path.
        :type return: np.ndarray of size [(our_robots):1] (distance)
        """
        final_distances = np.zeros(1, gameplay_node.NUM_ROBOTS / 2)
        for i in range(0, gameplay_node.NUM_ROBOTS / 2):
            distances = []
            for our_vec in our_vecs:
                if our_vec == np.zeros(1, 2) or our_vecs[i] == our_vec:
                    continue
                dot_over_mag = np.dot(our_vecs[i], our_vec) / (np.linalg.norm(our_vecs[i]) * np.linalg.norm(our_vec))
                if dot_over_mag < 0:
                    continue
                distances.append(np.linalg.norm(np.cross(our_vec, our_vecs[i])) / np.linalg.norm(our_vecs[i]))
            for their_vec in their_vecs:
                dot_over_mag = np.dot(our_vecs[i], their_vec) / (np.linalg.norm(our_vecs[i]) * np.linalg.norm(their_vec))
                if dot_over_mag < 0:
                    continue
                distances.append(np.linalg.norm(np.cross(their_vec, our_vecs[i])) / np.linalg.norm(our_vecs[i]))
            final_distances[i] = min(distances)
        return final_distances

    def best_goal_shot(self, world_state: stp.rc.WorldState, our_vecs: np.ndarray, their_vecs: np.ndarray) -> Tuple[np.ndarray, float]:
        """
        Finds the vector pointing to the area of the net that maximizes the distance from each robot to the path of the ball towards the net

        :param world_state: the current state of the world updated every tick
        :type world_state: stp.rc.WorldState
        :param our_vecs: the displacement vectors between our robot and our other teammates
        :type our_vecs: np.ndarray of size [(our_robots), 2]
        :param their_vecs: the displacement vectors between our robot and the other team's robots
        :type their_vecs: np.ndarray of size [(their_robots), 2]

        :return vector pointing from this robot to the best shot and the distance from the closest robot to the line spanning from this robot to the goal
        :type return: Tuple[np.ndarray [1, 2] (x, y), float]
        """
        pass