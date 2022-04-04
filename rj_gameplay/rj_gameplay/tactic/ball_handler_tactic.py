from ast import Pass
import math
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar
from rj_gameplay import gameplay_node
from rj_gameplay.role import ball_move, striker, passer
from rj_gameplay.skill.receive import Receive
from stp.rc import Robot, WorldState
from stp.role.cost import PickRobotById

import stp

from rj_gameplay.role import receiver, passer
import numpy as np

import stp.global_parameters as global_parameters

from rj_msgs.msg import RobotIntent


from enum import Enum, auto


class State(Enum):
    INIT = auto()
    INIT_TICK = auto()
    BALL_SECURE = auto()
    POSSESSING = auto()
    INIT_PASS = auto()
    INIT_PASS_TICK = auto()
    PASS = auto()
    AWAIT_PASSER_KICK = auto()
    PASS_IN_TRANSIT = auto()
    PASS_IN_TRANSIT_TICK = auto()
    INIT_RECEIVE = auto()
    INIT_RECEIVE_TICK = auto()
    RECEIVE = auto()
    INIT_SHOOT = auto()
    INIT_SHOOT_TICK = auto()
    SHOOT = auto()
    DONE = auto()

AGGRESSIVENESS = 1.01

# the angle cutoff for people intersepting passes or shots
CUTOFF_ANGLE = math.cos(math.pi / 10)

# NOTE: Change number on line 50 of capture.py back to 1.3

class BallHandlerTactic(stp.tactic.Tactic):
    def __init__(
        self,
        world_state: stp.rc.WorldState,
        init_cost: stp.role.cost,
        init_receiver_cost: stp.role.cost,
    ):
        super().__init__(world_state)

        self._state = State.INIT

        self._init_cost = init_cost
        self._init_receiver_cost = init_receiver_cost
        self.target_pass_robot = None

    def init_roles(self, world_state: stp.rc.WorldState) -> None:
        self.assigned_roles = []
        for i, robot in enumerate(self.assigned_robots):
            role = self._role_requests[i][1]
            if role is passer.PasserRole:
                self.assigned_roles.append(role(robot))
            elif role is receiver.ReceiverRole:
                self.assigned_roles.append(role(robot))
            elif role is striker.StrikerRole:
                self.assigned_roles.append(role(robot))
            elif role is ball_move.BallMoveRole:
                self.assigned_roles.append(role(robot))

    def tick(
        self, world_state: stp.rc.WorldState
    ) -> List[Tuple[int, RobotIntent]]:  # (id, intent)
        """
        FSM
         - init: request passer
         - on filled req: init_roles > tick passer's capture
         - when passer ready to pass: request receiver
         - on filled req: init_roles > tick passer pass
         - on ball passed: tick receiver, release passer role
         - when receiver done: done
        """
        print(self._state)

        role_intents = []

        if self._state == State.INIT:
            self._role_requests = [
                (
                    self._init_cost,
                    ball_move.BallMoveRole,
                )
            ]
            self._needs_assign = True
            self._state = State.INIT_TICK

        elif self._state == State.INIT_TICK:
            if len(self.assigned_roles) == 1:
                self.init_roles(world_state)
                self._state = State.BALL_SECURE

        elif self._state == State.BALL_SECURE:
            # TODO: these lines are a little ugly, any fix?
            ball_move_role = self.assigned_roles[0]
            intent = ball_move_role.tick(world_state)

            role_intents = [(ball_move_role.robot.id, intent)]

            if ball_move_role.ready:
                self._state = State.POSSESSING

        elif self._state == State.POSSESSING:
            ball_move_role = self.assigned_roles[0]
            intent = ball_move_role.tick(world_state)

            role_intents = [(ball_move_role.robot.id, intent)]

            # Calculate best shot and find nearest robot to the path and compare to threshold
            fake_striker = striker.StrikerRole(ball_move_role.robot)
            best_shot = fake_striker._find_target_point(world_state = world_state, kick_speed = 4.0)
            del(fake_striker)

            if self.check_kick(world_state, best_shot, ball_move_role.robot_id):
                #print("SHOOOOOOOOOOOOOTTTTTTING!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                self._state = State.INIT_SHOOT

            elif ball_move_role.is_done(world_state):
                self._state = State.INIT_SHOOT

            else:
                #pass ball to most forward person that can be passed to
                pass_targets = self.get_pass_order(world_state, ball_move_role.robot_id)
                for robot_id in pass_targets:
                    if self.check_kick(world_state, world_state.robots[robot_id].pose[0:2], ball_move_role.robot_id):
                        self._state = State.INIT_PASS
                        self.target_pass_robot = world_state.robots[robot_id]
                        break
        
        elif self._state == State.INIT_SHOOT:
            #print('init shoot')
            self._role_requests = [
                (
                    self._init_cost, 
                    striker.StrikerRole
                )
            ]

            self._needs_assign = True
            self._state = State.INIT_SHOOT_TICK

        elif self._state == State.INIT_SHOOT_TICK:
            #print('init shoot tick')
            if len(self.assigned_roles) == 1:
                self.init_roles(world_state)
                self._state = State.SHOOT

        elif self._state == State.INIT_PASS:

            self._role_requests = [
                (self._init_cost, passer.PasserRole),
                (PickRobotById(self.target_pass_robot.id), receiver.ReceiverRole)
            ]
            self._needs_assign = True
            self._state = State.INIT_PASS_TICK
            # decided to get receiver early to allow more coordination in theory, currently not written in
            # TODO: evaluate whether this is a dumb idea or not

        elif self._state == State.INIT_PASS_TICK:
            # Wait until play assignment assigns the needed 2 roles
            if len(self.assigned_roles) == 2:
                self.init_roles(world_state)
                self._state = State.PASS

        elif self._state == State.PASS:
            #print('passing')
            # TODO: these lines are a little ugly, any fix?
            passer_role = self.assigned_roles[0]
            receiver_role = self.assigned_roles[1]

            # TODO: create func to find good target point
            # TODO: should update receiver_role robot every tick in the role (see skill/capture.py)
            target_point = world_state.robots[receiver_role.robot.id].pose[0:2]
            passer_role.set_execute_pass(target_point)

            role_intents = [
                (passer_role.robot.id, passer_role.tick(world_state)),
                (receiver_role.robot.id, receiver_role.tick(world_state)),
            ]

            self._state = State.AWAIT_PASSER_KICK

        elif self._state == State.AWAIT_PASSER_KICK:
            # TODO: these lines are a little ugly, any fix?
            passer_role = self.assigned_roles[0]
            receiver_role = self.assigned_roles[1]

            role_intents = [
                (passer_role.robot.id, passer_role.tick(world_state)),
                (receiver_role.robot.id, receiver_role.tick(world_state)),
            ]

            if passer_role.is_done(world_state):
                self._state = State.PASS_IN_TRANSIT

        elif self._state == State.PASS_IN_TRANSIT:
            receiver_role = self.assigned_roles[1]
            print(receiver_role.robot.id)

            # TODO: here, we assume the initially picked Receiver is the best one
            # this may not be true (i.e. when intercept planning is bad and the ball does not get captured by the first receiver)
            # the receiver then should be reassigned to closest to ball
            # maybe check some is_done()?
            self._role_requests = [
                (
                    stp.role.cost.PickRobotById(receiver_role.robot.id),
                    receiver.ReceiverRole,
                )
            ]
            self._needs_assign = True
            self._state = State.PASS_IN_TRANSIT_TICK

        elif self._state == State.PASS_IN_TRANSIT_TICK:
            # Wait until play assignment assigns the needed 1 roles
            if len(self.assigned_roles) == 1:
                self.init_roles(world_state)
                self._state = State.INIT_RECEIVE

        elif self._state == State.INIT_RECEIVE:

            # TODO: these lines are a little ugly, any fix?
            receiver_role = self.assigned_roles[0]
            receiver_role.set_receive_pass()

            role_intents = [(receiver_role.robot.id, receiver_role.tick(world_state))]

            self._state = State.RECEIVE

        elif self._state == State.RECEIVE:
            receiver_role = self.assigned_roles[0]

            role_intents = [(receiver_role.robot.id, receiver_role.tick(world_state))]

            if receiver_role.is_done(world_state):
                self._state = State.DONE
                # end FSM

        elif self._state == State.SHOOT:
            #print('shooting')
            striker_role = self.assigned_roles[0]

            role_intents = [(striker_role.robot.id, striker_role.tick(world_state))]

            if striker_role.is_done(world_state):
                self._state = State.DONE


        return role_intents

    @property
    def needs_assign(self):
        # style is wrong here: this convoluted way of returning self._needs_assign is necessary because we want to set it to False after the call, always
        ret = self._needs_assign == True  # noqa: E712
        self._needs_assign = False
        return ret

    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        return self._state == State.DONE

    def check_kick(self, world_state: stp.rc.WorldState, target: np.ndarray, robot_id: int) -> bool:
        """
        True -> Good Shot/Pass
        False -> Bad Shot/Pass
        """
        relatate_vector = lambda target_pos: target_pos - world_state.robots[robot_id].pose[0:2]
        relative_vectors = [relatate_vector(robot.pose[0:2]) for robot in world_state.robots]

        cosine = lambda robot_pos: (np.dot(robot_pos, target)) / (np.linalg.norm(robot_pos) * np.linalg.norm(target)) if (np.linalg.norm(robot_pos) != 0 and np.linalg.norm(target) != 0) else 1
        if max([cosine(relative_vector) for relative_vector in relative_vectors]) > CUTOFF_ANGLE:
            return False
        return True

    def get_pass_order(self, world_state: stp.rc.WorldState, robot_id) -> List[Robot]:
        """
        
        """
        dist_from_goal = lambda robot: np.linalg.norm(robot.pose[0:2] - world_state.field.their_goal_loc)
        robot_distances = {robot.id : dist_from_goal(robot) for robot in world_state.our_robots}

        robot_distances.pop(robot_id)
        robot_distances.pop(world_state.goalie_id)

        robot_distances = {robot : distance for robot, distance in sorted(robot_distances.items(), key=lambda item: item[1])}

        return robot_distances.keys()

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
            if (world_state.robots[i].id != self.robot.id and world_state.robots[i].id != world_state.goalie_id):
                our_vecs[i,0:2] = world_state.robots[i].pose[0:2] - self.robot.pose[0:2]
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
                final_distances[0, i] = min(distances) * (world_state.robots[i].pose[1] * AGGRESSIVENESS)
        return final_distances

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
