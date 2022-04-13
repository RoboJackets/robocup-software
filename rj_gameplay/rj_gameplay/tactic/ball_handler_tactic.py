import math
from enum import Enum, auto
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar

import numpy as np
import stp
import stp.global_parameters as global_parameters
from rj_msgs.msg import RobotIntent
from stp.rc import Robot, WorldState
from stp.role.cost import PickRobotById

from rj_gameplay import gameplay_node
from rj_gameplay.role import ball_move, passer, receiver, striker
from rj_gameplay.skill.receive import Receive


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
CUTOFF_VALUE = 0.9

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
        if self._state != State.DONE:
            print(self._state)

        role_intents = []
        self._init_cost = stp.role.cost.PickClosestToPoint(world_state.ball.pos)

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

            # Calculate best shot and find nearest robot to the path and compare to threshold
            fake_striker = striker.StrikerRole(ball_move_role.robot)
            best_shot = fake_striker._find_target_point(
                world_state=world_state, kick_speed=5.0
            )
            del fake_striker

            print("Checking Shot")
            if self.check_kick(world_state, best_shot, ball_move_role.robot_id):
                self._state = State.INIT_SHOOT

            elif self.get_best_pass(world_state, ball_move_role.robot_id):
                self._state = State.INIT_PASS

            elif ball_move_role.is_done(world_state):
                print("Can't dribble anymore")
                self._state = State.INIT_SHOOT

            else:
                print("Dribbling")
                intent = ball_move_role.tick(world_state)
                role_intents = [(ball_move_role.robot.id, intent)]

        elif self._state == State.INIT_SHOOT:
            # print('init shoot')
            self._role_requests = [(self._init_cost, striker.StrikerRole)]

            self._needs_assign = True
            self._state = State.INIT_SHOOT_TICK

        elif self._state == State.INIT_SHOOT_TICK:
            # print('init shoot tick')
            if len(self.assigned_roles) == 1:
                self.init_roles(world_state)
                self._state = State.SHOOT

        elif self._state == State.SHOOT:
            striker_role = self.assigned_roles[0]
            role_intents = [(striker_role.robot.id, striker_role.tick(world_state))]
            if striker_role.is_done(world_state):
                self._state = State.DONE
                # end FSM

        elif self._state == State.INIT_PASS:

            self._role_requests = [
                (self._init_cost, passer.PasserRole),
                (PickRobotById(self.target_pass_robot.id), receiver.ReceiverRole),
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
            # print('passing')
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
            # print(receiver_role.robot.id)

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

        return role_intents

    @property
    def needs_assign(self):
        # style is wrong here: this convoluted way of returning self._needs_assign is necessary because we want to set it to False after the call, always
        ret = self._needs_assign == True  # noqa: E712
        self._needs_assign = False
        return ret

    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        return self._state == State.DONE

    def get_best_pass(
        self, world_state: stp.rc.WorldState, passer_robot_id: int
    ) -> bool:
        print("Checking Passes")
        pass_targets = self.get_pass_order(world_state, passer_robot_id)
        for robot_id in pass_targets:
            if self.check_kick(
                world_state,
                world_state.robots[robot_id].pose[0:2],
                passer_robot_id,
            ):
                self.target_pass_robot = world_state.robots[robot_id]
                return True

        return False

    def check_kick(
        self, world_state: stp.rc.WorldState, target: np.ndarray, robot_id: int
    ) -> bool:
        """
        True -> Good Shot/Pass
        False -> Bad Shot/Pass
        """
        ball_dir = lambda pos: pos - world_state.ball.pos
        ball_dir_norm = lambda pos: ball_dir(pos) / np.linalg.norm(ball_dir(pos))
        relative_vectors = [
            ball_dir_norm(robot.pose[0:2])
            for robot in world_state.their_robots
            if robot.visible
        ]

        print("Target Position: ", target)
        target = ball_dir_norm(target)
        print("Normalized Target Position: ", target)

        kick_values = [np.dot(vector, target) for vector in relative_vectors]
        kick_values_dict = {
            str(vector): np.dot(vector, target) for vector in relative_vectors
        }
        print("Kick Values:", kick_values_dict)

        if max(kick_values) > CUTOFF_VALUE:
            return False
        return True

    def get_pass_order(
        self, world_state: stp.rc.WorldState, robot_id: int
    ) -> List[Robot]:
        """ """
        dist_from_goal = lambda robot: np.linalg.norm(
            robot.pose[0:2] - world_state.field.their_goal_loc
        )
        robot_distances = {
            robot.id: dist_from_goal(robot) for robot in world_state.our_robots
        }

        robot_distances.pop(robot_id)
        robot_distances.pop(world_state.goalie_id)

        robot_distances = {
            robot: distance
            for robot, distance in sorted(
                robot_distances.items(), key=lambda item: item[1]
            )
        }

        return list(robot_distances.keys())

    def _blocker_margin(
        self,
        kick_origin: np.array,
        kick_target: np.array,
        kick_speed: float,
        blocker: stp.rc.Robot,
    ):
        if not blocker.visible:
            return np.inf

        kick_vector = kick_target - kick_origin
        kick_dist = np.linalg.norm(kick_vector)
        kick_vector /= kick_dist
        # unused ??
        # kick_perp = np.array([kick_vector[1], -kick_vector[0]])

        blocker_position = blocker.pose[0:2]

        # Calculate blocker intercept
        blocker_intercept_dist_along_kick = np.dot(
            blocker_position - kick_origin, kick_vector
        )
        blocker_intercept_dist_along_kick = np.clip(
            blocker_intercept_dist_along_kick, a_min=0, a_max=kick_dist
        )
        blocker_intercept = (
            kick_origin + kick_vector * blocker_intercept_dist_along_kick
        )

        blocker_distance = np.clip(
            np.linalg.norm(blocker_intercept - blocker_position) - EFF_BLOCK_WIDTH,
            a_min=0.0,
            a_max=np.inf,
        )

        blocker_time = np.abs(blocker_distance) / OPPONENT_SPEED

        # Doesn't include friction...oops?
        ball_time = np.linalg.norm(blocker_intercept - kick_origin) / kick_speed

        return blocker_time - ball_time

    def _kick_cost(
        self,
        point: np.array,
        kick_speed: float,
        kick_origin: np.array,
        world_state: stp.rc.WorldState,
    ):
        margins = [
            self._blocker_margin(kick_origin, point, kick_speed, blocker)
            for blocker in world_state.their_robots
        ]
        return -min(margins)

    def _find_target_point(
        self, world_state: stp.rc.WorldState, kick_speed: float
    ) -> np.ndarray:
        goal_y = world_state.field.length_m
        cost = 0

        ball_pos = world_state.ball.pos

        # Heuristic: limit where we kick if we're very wide
        xmin = -0.43
        xmax = 0.43
        if abs(ball_pos[0]) > 1:
            kick_extent = -1 / ball_pos[0]
            if kick_extent < 0:
                xmin = np.clip(kick_extent, a_min=xmin, a_max=0)
            elif kick_extent > 0:
                xmax = np.clip(kick_extent, a_min=0, a_max=xmax)

        try_points = [np.array([x, goal_y]) for x in np.arange(xmin, xmax, step=0.05)]

        cost, point = min(
            [
                (
                    self._kick_cost(
                        point, kick_speed, world_state.ball.pos, world_state
                    ),
                    point,
                )
                for point in try_points
            ],
            key=lambda x: x[0],
        )

        return point
