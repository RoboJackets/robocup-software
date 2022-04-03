from ast import Pass
import math
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar
from rj_gameplay import gameplay_node
from rj_gameplay.role import ball_move, striker, passer, receiver
from rj_gameplay.skill.receive import Receive
from stp.rc import Robot, WorldState
from stp.role.cost import PickRobotById

import stp

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
            best_shot = striker.StrikerRole()._find_target_point(world_state = world_state, kick_speed = 4.0)

            if self.check_kick(world_state, best_shot, ball_move_role.robot_id):
                print("SHOOOOOOOOOOOOOTTTTTTING!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                self._state = State.INIT_SHOOT

            elif ball_move_role.is_done(world_state):
                self._state = State.INIT_SHOOT

            else:
                #pass ball to most forward person that can be passed to
                pass_targets = self.get_pass_order(world_state, ball_move_role.robot_id)
                for robot in pass_targets:
                    if self.check_kick(world_state, robot.pose[0:2], ball_move_role.robot_id):
                        self._state = State.INIT_PASS
                        self.target_pass_robot = robot
                        break
        
        elif self._state == State.INIT_SHOOT:
            print('init shoot')
            self._role_requests = [
                (
                    self._init_cost, 
                    striker.StrikerRole
                )
            ]

            self._needs_assign = True
            self._state = State.INIT_SHOOT_TICK

        elif self._state == State.INIT_SHOOT_TICK:
            print('init shoot tick')
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
            print('passing')
            # TODO: these lines are a little ugly, any fix?
            passer_role = self.assigned_roles[0]
            receiver_role = self.assigned_roles[1]

            # TODO: create func to find good target point
            # TODO: should update receiver_role robot every tick in the role (see skill/capture.py)
            target_point = world_state.our_robots[receiver_role.robot.id].pose[0:2]
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
            print('shooting')
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
        relatate_vector = lambda target_pos: target_pos - world_state.our_robots[robot_id].pose[0:2]
        relative_vectors = [relatate_vector(robot.pose[0:2]) for robot in world_state.robots]

        cosine = lambda robot_pos: (np.dot(robot_pos, target)) / (np.linalg.norm(robot_pos) * np.linalg.norm(target))
        if max([cosine(relative_vector) for relative_vector in relative_vectors]) > CUTOFF_ANGLE:
            return False
        return True

    def get_pass_order(self, world_state: stp.rc.WorldState, robot_id) -> List[Robot]:
        """
        
        """
        dist_from_goal = lambda robot: np.linalg.norm(robot.pose[0:2] - world_state.field.their_goal_loc)
        robot_distances = {robot.id : dist_from_goal(robot) for robot in world_state.our_robots}

        robot_distances.pop(world_state.our_robots[robot_id])
        robot_distances.pop(world_state.our_robots[world_state.goalie_id])

        robot_distances = {robot : distance for robot, distance in sorted(robot_distances.items(), key=lambda item: item[1])}

        return robot_distances.keys()
