from enum import Enum, auto
from lzma import is_check_supported

from soupsieve import closest

import stp.role
import stp.rc
from stp.rc import Ball, Robot, WorldState

from typing import List, TypedDict, Tuple

from rj_gameplay.skill import receive, pivot_kick, dribble  # , line_kick
import rj_gameplay.gameplay_node as gameplay_node

from rj_msgs.msg import RobotIntent

import numpy as np

#TODO add reward to getting to a better shot down the field

class State(Enum):
    # Initialization
    INIT = auto()
    # Robot is settling ball
    RECEIVING = auto()
    # Robot has ball and is considering options
    POSSESSING = auto()
    # Robot has identified pass and is passing
    PASSING = auto()
    # Robot has identified shot on goal and is shooting
    SHOOTING = auto()
    # Kick has been completed
    KICK_DONE = auto()

class BallReleaseState(Enum):
    # Robot does not plan to get rid of ball
    HOLDING = auto()
    # Robot is getting ready to pass or shoot
    INIT = auto()
    # Robot is executing pass or shoot
    EXECUTING = auto()
    # Robot no longer has the ball
    FINALIZED = auto()

class DribbleState(Enum):
    # Robot is not moving
    CHILLING = auto()
    # Robot is moving
    DRIBBLING = auto()


# Constant for how far a robot can be from the pass line before stopping the pass from occuring
PASS_DISTANCE_CUTOFF = 0.5
# Constant for how far from the path of the ball the closest enemy robot can be
SHOOT_DISTANCE_CUTOFF = 6
# Number of divisions for finding the best shot on goal
NUM_DIVISIONS = 15
# Aggressiveness hyperparameter alters the distance robots are from the robot the farther down the field a robot is (basically aggressiveness weights how
# much this robot wants to pass down the field)
AGGRESSIVENESS = 1.01
# The amount of distance the robot can move
MOVE_CUTOFF = 0.8
# The robot should pass if there is pressure on it
PRESSURE_CUTOFF_DIST = 0.5

class PasserRole(stp.role.Role):
    def __init__(self, robot: stp.rc.Robot) -> None:
        super().__init__(robot)

        self.receive_skill = None
        self.pivot_kick_skill = None
        self.dribble_skill = None

        self.receive_location = None

        # TODO: make FSM class (or at least use enum instead of str literals)
        self._state = State.INIT
        self._ball_state = BallReleaseState.HOLDING
        self.__dribble_state = DribbleState.CHILLING
        self._target_robot = None

        self._target_point = None

        self.left_goal_post = None
        
        self.right_goal_post = None
        self.shot_on_goal = (np.zeros((1, 2)), 0)

        self.rob_pass_dists = []
        self.dist_closest_enemy = np.inf

    @property
    def pass_ready(self):
        return self._state == State.POSSESSING

    def set_execute_pass(self, target_point):
        pass

    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
        """
        Assume robot does not have ball on init. Then:
         - on init: get ball
         - when got ball: mark pass ready for Tactic, dribble, wait
         - on pass signal from Tactic: pivot_kick to point, let receiver get ball, done
        """

        intent = None
        print(self._state)
        print(gameplay_node.NUM_ROBOTS_PER_TEAM)

        # Initialization of pass
        if self._state == State.INIT:
            print("initializing")
            # Set up goal_posts
            if self.left_goal_post is None:
                self.left_goal_post = world_state.field.their_goal_loc
                self.left_goal_post[0] -= world_state.field.goal_width_m / 2
                self.right_goal_post = world_state.field.their_goal_loc
                self.right_goal_post[0] += world_state.field.goal_width_m / 2

            # Begin receiving ball
            self.receive_skill = receive.Receive(robot=self.robot)
            intent = self.receive_skill.tick(world_state)
            self._state = State.RECEIVING

        # Receive the ball
        elif self._state == State.RECEIVING:
            print("recieving")
            intent = self.receive_skill.tick(world_state)
            if self.receive_skill.is_done(world_state):
                print("done receiving")
                self._state = State.POSSESSING
                print("possessing pt 1")
                self.receive_location = self.robot.pose[0:2]

        # This Robot has the ball
        elif self._state == State.POSSESSING:
            print("posessing")
            # Initialize values
            our_vecs, their_vecs = self.calc_displacement_vecs(world_state)
            self.rob_pass_dists = self.calc_min_distance_from_lines(world_state, our_vecs, their_vecs)
            self.shot_on_goal = self.best_goal_shot(world_state, NUM_DIVISIONS, self.left_goal_post, self.right_goal_post)
            best_pass_index = self.get_best_pass_index(self.rob_pass_dists[0])
            move_distance = np.linalg.norm(self.robot.pose[0:2] - self.receive_location)
            closest_defender_distance = self.get_closest_enemy_dist(their_vecs)

            print(self.shot_on_goal[1])
            print(closest_defender_distance)
            
            # Hanlde State Transitions #
            print("best shot: {}".format(self.shot_on_goal[1]))
            print("best pass: {}".format(self.rob_pass_dists[0][best_pass_index]))
            if self.shot_on_goal[1] > SHOOT_DISTANCE_CUTOFF:
                self._target_point = self.shot_on_goal[0]
                self._ball_state = BallReleaseState.INIT
                self._state = State.SHOOTING

            elif closest_defender_distance < PRESSURE_CUTOFF_DIST:
                if self.rob_pass_dists[0][best_pass_index] > PASS_DISTANCE_CUTOFF:
                    self._target_robot = world_state.our_robots[best_pass_index]
                    self._ball_state = BallReleaseState.INIT
                    self._state = State.PASSING

            elif move_distance > MOVE_CUTOFF:
                self._target_robot = world_state.our_robots[best_pass_index]
                self._ball_state = BallReleaseState.INIT
                self._state = State.PASSING

            elif self.__dribble_state != DribbleState.DRIBBLING:
                #print("I do be dribbling")
                #self.dribble_skill = dribble.Dribble(
                    #robot=self.robot,
                    #arget_point=(self.left_goal_post + self.right_goal_post) / 2,
                    #target_vel=np.array([0, 1])
                #)
                #self.__dribble_state = DribbleState.DRIBBLING
                pass

            else:
                #intent = self.dribble_skill.tick(world_state, intent)
                pass
            
        # Robot is in passing state
        elif self._state == State.PASSING:
            print("passing")
            # TODO: make these params configurable

            # Initialize pass
            print(self._ball_state)
            if self._ball_state == BallReleaseState.INIT:
                self._target_point = self._target_robot.pose[0:2]
                self.pivot_kick_skill = pivot_kick.PivotKick(
                    robot=self.robot,
                    target_point=self._target_point,
                    chip=False,
                    kick_speed=4.0,  # TODO: adjust based on dist from target_point
                )
                self._ball_state = BallReleaseState.EXECUTING

            # Executing pass
            if self._ball_state == BallReleaseState.EXECUTING:
                print('executing passing')
                intent = self.pivot_kick_skill.tick(world_state)

                if self.pivot_kick_skill.is_done(world_state):
                    self._ball_state = BallReleaseState.FINALIZED

        # Robot is in shooting state
        elif self._state == State.SHOOTING:
            print("shooting")
            
            # Initialize shot
            if self._ball_state == BallReleaseState.INIT:
                self.pivot_kick_skill = pivot_kick.PivotKick(
                    robot=self.robot,
                    target_point=self._target_point,
                    chip=False,
                )
                self._ball_state = BallReleaseState.EXECUTING

            # Executing shot
            if self._ball_state == BallReleaseState.EXECUTING:
                intent = self.pivot_kick_skill.tick(world_state)

                if self.pivot_kick_skill.is_done(world_state):
                    self._ball_state = BallReleaseState.FINALIZED

        return intent

    def is_done(self, world_state) -> bool:
        return self._ball_state == BallReleaseState.FINALIZED

    def calc_displacement_vecs(self, world_state: stp.rc.WorldState) -> Tuple[np.ndarray]:
        """
        Calculates the vector between the robot assigned the passer role and the other robots on the field.  For the case where the robot is checking against
        it's own position, the vector given is (0, 0).  The goalie is also given the vector (0, 0)

        :param world_state: the world_state representation we have each tick
        :type world_state: stp.rc.WorldState

        :return [(our_robots), 3] array corresponding to the displacement vector (x, y, 0) this robot to each of our other robots and a second array between this robot
        and the other teams robots
        :type return: 2 np.ndarray of dimension [(NUM_ROBOTS / 2), 3]
        """
        our_vecs = np.zeros((gameplay_node.NUM_ROBOTS_PER_TEAM, 2))
        for i in range(0, gameplay_node.NUM_ROBOTS_PER_TEAM):
            if (world_state.our_robots[i].id != self.robot.id and world_state.our_robots[i].id != world_state.goalie_id):
                our_vecs[i,0:2] = world_state.our_robots[i].pose[0:2] - self.robot.pose[0:2]
        their_vecs = np.zeros((gameplay_node.NUM_ROBOTS_PER_TEAM, 2))
        for j in range(0, gameplay_node.NUM_ROBOTS_PER_TEAM):
            their_vecs[j,0:2] = world_state.their_robots[j].pose[0:2] - self.robot.pose[0:2]
        return our_vecs, their_vecs

    def calc_min_distance_from_lines(self, world_state: WorldState, our_vecs: np.ndarray, their_vecs: np.ndarray) -> np.ndarray:
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
        final_distances = np.zeros((1, gameplay_node.NUM_ROBOTS_PER_TEAM))
        for i in range(0, gameplay_node.NUM_ROBOTS_PER_TEAM):
            distances = []
            if our_vecs[i][0] == 0 and our_vecs[i][1] == 0:
                continue
            for our_vec in our_vecs:
                if our_vec[0] == 0 and our_vec[1] == 0:
                    continue
                if our_vecs[i][0] == our_vec[0] and our_vecs[i][1] == our_vec[1]:
                    continue
                dot_over_mag = np.dot(our_vecs[i], our_vec) / (np.linalg.norm(our_vecs[i]) * np.linalg.norm(our_vec))
                if dot_over_mag < 0:
                    continue
                dist = np.linalg.norm(np.cross(our_vec, our_vecs[i])) / np.linalg.norm(our_vecs[i])
                distances.append(dist)
            robot_num = 0
            for their_vec in their_vecs:
                robot_num += 1
                dot_over_mag = np.dot(our_vecs[i], their_vec) / (np.linalg.norm(our_vecs[i]) * np.linalg.norm(their_vec))
                if dot_over_mag < 0:
                    continue
                dist = np.linalg.norm(np.cross(our_vec, our_vecs[i])) / np.linalg.norm(our_vecs[i])
                distances.append(dist)
            if len(distances) is not 0:
                final_distances[0, i] = min(distances) * (world_state.our_robots[i].pose[1] * AGGRESSIVENESS)
        return final_distances

    def best_goal_shot(self, world_state: stp.rc.WorldState, divisions: int, left_post: np.ndarray, right_post: np.ndarray) -> Tuple[np.ndarray, float]:
        """
        Finds the vector pointing to the area of the net that maximizes the distance from each robot to the path of the ball towards the net

        :param world_state: the current state of the world updated every tick
        :type world_state: stp.rc.WorldState
        :param divisions: the number of divisions to split the goal into to find the best shot
        :type divisions: int
        :param left_post: the location of the left post of the goal
        :type left_post: np.ndarray of size [1, 2] (x, y)
        :param right_post: the location of the right post of the goal
        :type right_post: np.ndarray of size [1, 2] (x, y)

        :return vector pointing from this robot to the best shot and the distance from the closest robot to the line spanning from this robot to the goal
        :type return: Tuple[np.ndarray [1, 2] (x, y), float]
        """
        largest_gap = 0
        best_shot_location = np.zeros((1, 2))

        front_robs = []
        for robot in world_state.robots:
            if robot.pose[1] > self.robot.pose[1]:
                front_robs.append(robot)

        delta = (right_post - left_post) / divisions

        for i in range(0, divisions):
            shot_position = left_post + (delta * i)
            shot_constant = shot_position - self.robot.pose[0:2]
            closest_value = 0
            closest_index = 0

            index = 0
            for robot in front_robs:
                value = robot.pose[1] * shot_constant[0] - robot.pose[0] * shot_constant[1]
                if value < closest_value:
                    closest_value = value
                    closest_index = index
                index += 1
            
            distance = np.linalg.norm(np.cross(self.robot.pose[0:2], (front_robs[closest_index].pose[0:2] - self.robot.pose[0:2]))) / np.linalg.norm(self.robot.pose[0:2])
            if distance > largest_gap:
                largest_gap = distance
                best_shot_location = shot_position

        return best_shot_location, largest_gap

    def get_best_pass_index(self, robot_pass_distances: np.ndarray) -> int:
        """
        Finds the index of the best robot to pass to.

        :param robot_pass_distances: a np.ndarray of possible robots to pass to and the shortest distance
        from another robot to the line passing to the robot.
        :type robot_pass_distances: np.ndarray of shape ((our_robots), 1)

        :return the index of the best pass
        :type return: int
        """
        max_distance = 0
        max_distance_index = 0

        for i in range(0, robot_pass_distances.size):
            if robot_pass_distances[i] > max_distance:
                max_distance = robot_pass_distances[i]
                max_distance_index = i

        return max_distance_index

    def get_closest_enemy_dist(self, their_robots: np.ndarray) -> float:
        """
        Finds the distance from this robot to the closest enemy robot.

        :param their_robots: the displacement vector between this robot and the opponenets robots.
        :type their_robots: np.ndarray of size ((their_robots), 2)

        :return the minimum distance from another robot to this robot
        :type return: float
        """
        min_distance = np.inf
        for i in range(0, their_robots.size // 2):
            distance = np.linalg.norm(their_robots[i])
            if distance < min_distance:
                min_distance = distance
        return min_distance